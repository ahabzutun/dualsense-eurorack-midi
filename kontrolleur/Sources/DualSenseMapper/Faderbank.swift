import SwiftUI
import Observation

// MARK: - Model

struct Fader: Equatable, Codable {
    var usb_channel = 16
    var usb_cc = 80
    var usb_14bit = false
    var trs_channel = 1
    var trs_cc = 32
    var trs_14bit = false
    var curve = 64
}

struct FaderOptions: Equatable, Codable {
    var led_on = true
    var led_flash = true
    var rotate = false
    var i2c_leader = false
    var midi_thru = false
    var fader_min = 15
    var fader_max = 8135
}

struct FaderbankDTO: Decodable {
    let ok: Bool
    let error: String?
    let device: String?
    let firmware: String?
    let hires_supported: Bool?
    let curves_supported: Bool?
    let options: FaderOptions?
    let faders: [Fader]?
    let port: String?
}

struct FaderInstallDTO: Decodable {
    let ok: Bool
    let error: String?
    let verified: Bool?
    let mismatch: [String]?
    let skipped: [String]?
}

enum Curve {
    /// out = in ^ (2 ^ ((v − 64) / 16)) — same formula as the 16n firmware fork
    static func exponent(_ v: Int) -> Double { pow(2.0, Double(v - 64) / 16.0) }
    static func apply(_ x: Double, _ v: Int) -> Double { pow(max(0, min(1, x)), exponent(v)) }
    static func name(_ v: Int) -> String {
        if v == 64 { return "Linear" }
        let k = exponent(v)
        return v > 64 ? String(format: "Exp  x^%.1f", k) : String(format: "Log  x^%.2f", k)
    }
    static let presets: [(String, Int)] = [("Log+", 32), ("Log", 48), ("Lin", 64), ("Exp", 80), ("Exp+", 96)]
}

@MainActor @Observable
final class Faderbank {
    var faders: [Fader] = (0..<16).map { Fader(usb_cc: 80 + $0, trs_cc: 32 + $0) }
    var options = FaderOptions()
    var loaded = false
    var device = ""
    var firmware = ""
    var curvesSupported = false
    var hiresSupported = false
    var port = ""

    func apply(_ d: FaderbankDTO) {
        if let f = d.faders, f.count == 16 { faders = f }
        if let o = d.options { options = o }
        device = d.device ?? ""
        firmware = d.firmware ?? ""
        curvesSupported = d.curves_supported ?? false
        hiresSupported = d.hires_supported ?? false
        port = d.port ?? ""
        loaded = true
    }

    var json: String {
        struct Out: Encodable { let faders: [Fader]; let options: FaderOptions }
        let enc = JSONEncoder(); enc.outputFormatting = [.sortedKeys]
        return String(decoding: (try? enc.encode(Out(faders: faders, options: options))) ?? Data(), as: UTF8.self)
    }

    var problems: [String] {
        var p: [String] = []
        for (k, f) in faders.enumerated() {
            if f.usb_14bit && f.usb_cc >= 96 { p.append("Fader \(k + 1): 14-bit USB needs a CC below 96 (the LSB goes to CC + 32).") }
            if f.trs_14bit && f.trs_cc >= 96 { p.append("Fader \(k + 1): 14-bit TRS needs a CC below 96.") }
        }
        if options.fader_min >= options.fader_max { p.append("Fader min must be below fader max.") }
        return p
    }
}

// MARK: - Running the helper (on the Pi over ssh, or here on the Mac)

struct FaderClient {
    var onPi: Bool
    var pi: PiClient
    var python: String          // local interpreter with python-rtmidi
    var repo: String

    enum Failure: LocalizedError {
        case msg(String)
        var errorDescription: String? { if case .msg(let s) = self { return s }; return nil }
    }

    private var localScript: String? { Bundle.main.path(forResource: "fader16_tool", ofType: "py") }

    private func runLocal(_ arg: String, input: String?) async throws -> String {
        guard let script = localScript else { throw Failure.msg("fader16_tool.py missing in the app bundle") }
        let python = python
        return try await withCheckedThrowingContinuation { cont in
            DispatchQueue.global(qos: .userInitiated).async {
                let p = Process()
                p.executableURL = URL(fileURLWithPath: (python as NSString).expandingTildeInPath)
                p.arguments = [script, arg]
                let out = Pipe(), err = Pipe(), inp = Pipe()
                p.standardOutput = out; p.standardError = err; p.standardInput = inp
                do { try p.run() } catch { cont.resume(throwing: Failure.msg("can't start \(python): \(error.localizedDescription)")); return }
                if let input, let d = input.data(using: .utf8) { inp.fileHandleForWriting.write(d) }
                try? inp.fileHandleForWriting.close()
                let o = out.fileHandleForReading.readDataToEndOfFile()
                let e = err.fileHandleForReading.readDataToEndOfFile()
                p.waitUntilExit()
                let so = String(decoding: o, as: UTF8.self)
                cont.resume(returning: so.isEmpty ? String(decoding: e, as: UTF8.self) : so)
            }
        }
    }

    private func decode<T: Decodable>(_ text: String) throws -> T {
        guard let line = text.split(separator: "\n").last(where: { $0.hasPrefix("{") }), let data = line.data(using: .utf8) else {
            throw Failure.msg(text.isEmpty ? "no answer from fader16_tool" : String(text.suffix(300)))
        }
        return try JSONDecoder().decode(T.self, from: data)
    }

    func export() async throws -> FaderbankDTO {
        let text = onPi ? try await pi.runRemote("python3 \(repo)/scripts/fader16_tool.py export")
                        : try await runLocal("export", input: nil)
        return try decode(text)
    }

    func install(_ json: String) async throws -> FaderInstallDTO {
        let text = onPi ? try await pi.runRemote("python3 \(repo)/scripts/fader16_tool.py install", input: json)
                        : try await runLocal("install", input: json)
        return try decode(text)
    }
}

// MARK: - Faderbank drawing

@MainActor
struct FaderbankCanvas: View {
    @Environment(Faderbank.self) private var bank
    @Environment(Mapping.self) private var mapping
    @Binding var selected: Int

    var body: some View {
        ZoomStage(size: CGSize(width: 1100, height: 640)) {
            ZStack(alignment: .topLeading) {
                BlueprintGrid()
                RoundedRectangle(cornerRadius: 22)
                    .fill(LinearGradient(colors: [Theme.body.opacity(0.10), Theme.body.opacity(0.03)], startPoint: .top, endPoint: .bottom))
                    .overlay(RoundedRectangle(cornerRadius: 22).stroke(Theme.body.opacity(0.55), lineWidth: 1.6))
                    .frame(width: 1000, height: 520)
                    .position(x: 550, y: 330)
                Text(bank.loaded ? "16n · \(bank.device) · firmware \(bank.firmware)\(bank.curvesSupported ? " + curves" : "")" : "16n · not loaded")
                    .font(Theme.font(11, .heavy, .mono)).tracking(1.2)
                    .foregroundStyle(Theme.dim)
                    .position(x: 550, y: 52)
                HStack(spacing: 4) {
                    ForEach(0..<16, id: \.self) { i in
                        FaderColumn(index: i, selected: selected == i)
                            .onTapGesture { selected = i }
                    }
                }
                .frame(width: 980)
                .position(x: 550, y: 330)
            }
        }
    }
}

@MainActor
struct FaderColumn: View {
    @Environment(Faderbank.self) private var bank
    @Environment(Mapping.self) private var mapping
    @Environment(NotesStore.self) private var notes
    let index: Int
    let selected: Bool

    var body: some View {
        let f = bank.faders[index]
        let port = mapping.faderPort(channel: f.usb_channel, cc: f.usb_cc)
        let col: Color = f.curve == 64 ? Theme.cv : Theme.function
        VStack(spacing: 8) {
            Text("\(index + 1)").font(Theme.font(13, .heavy, .mono))
                .foregroundStyle(selected ? Theme.ink : Theme.ink.opacity(0.8))
            CurvePreview(curve: f.curve, color: col)
                .frame(width: 44, height: 34)
            ZStack {
                Capsule().fill(Theme.bg.opacity(0.8)).frame(width: 8, height: 250)
                Capsule().stroke(Theme.body.opacity(0.35), lineWidth: 1).frame(width: 8, height: 250)
                RoundedRectangle(cornerRadius: 4)
                    .fill(selected ? Theme.ink.opacity(0.9) : col.opacity(0.35))
                    .overlay(RoundedRectangle(cornerRadius: 4).stroke(col, lineWidth: 1.4))
                    .overlay(Rectangle().fill(Theme.bg).frame(height: 2))
                    .frame(width: 36, height: 22)
                    .offset(y: 40)
            }
            .frame(height: 256)
            VStack(spacing: 3) {
                Text("ch\(f.usb_channel)").font(Theme.font(10, .semibold, .mono)).foregroundStyle(Theme.dim)
                Text("CC\(f.usb_cc)\(f.usb_14bit ? "ʰ" : "")").font(Theme.font(11, .bold, .mono))
                    .foregroundStyle(Theme.ink)
                Text(port.map { "CV \($0)" } ?? "—")
                    .font(Theme.font(10, .bold, .mono))
                    .foregroundStyle(port == nil ? Theme.dim : Theme.cv)
                    .padding(.horizontal, port == nil ? 0 : 5).padding(.vertical, 2)
                    .background(Capsule().fill(port == nil ? Color.clear : Theme.cv.opacity(0.14)))
                if let n = notes.note(NotesStore.fader(index)) {
                    NoteBadge(text: n, size: 8.5, lines: 2).frame(width: 56)
                }
            }
        }
        .padding(.vertical, 12)
        .frame(width: 56)
        .background(RoundedRectangle(cornerRadius: 10).fill(selected ? col.opacity(0.10) : Color.clear))
        .overlay(RoundedRectangle(cornerRadius: 10).stroke(selected ? Theme.ink.opacity(0.7) : Color.clear, lineWidth: 1.2))
        .shadow(color: selected ? col.opacity(0.6) : .clear, radius: 10)
        .contentShape(Rectangle())
    }
}

struct CurvePreview: View {
    let curve: Int
    var color: Color = Theme.cv
    var body: some View {
        Canvas { ctx, size in
            var frame = Path(); frame.addRect(CGRect(origin: .zero, size: size))
            ctx.stroke(frame, with: .color(Theme.body.opacity(0.2)), lineWidth: 0.6)
            var p = Path()
            for s in 0...40 {
                let x = Double(s) / 40
                let pt = CGPoint(x: x * size.width, y: (1 - Curve.apply(x, curve)) * size.height)
                if s == 0 { p.move(to: pt) } else { p.addLine(to: pt) }
            }
            ctx.stroke(p, with: .color(color), lineWidth: 1.6)
        }
    }
}

// MARK: - Inspector

enum FaderTab: String, CaseIterable, Identifiable {
    case fader = "Fader", device = "Device"
    var id: String { rawValue }
}

@MainActor
struct FaderPanel: View {
    @Environment(Faderbank.self) private var bank
    @Environment(Mapping.self) private var mapping
    @Environment(NotesStore.self) private var notes
    @Binding var selected: Int
    @State private var tab: FaderTab = .fader

    var body: some View {
        VStack(spacing: 0) {
            Picker("", selection: $tab) { ForEach(FaderTab.allCases) { Text($0.rawValue).tag($0) } }
                .pickerStyle(.segmented).labelsHidden().padding(14)
            Divider().overlay(Theme.line)
            ScrollView {
                Group {
                    if tab == .fader { faderEditor } else { deviceEditor }
                }
                .padding(14)
            }
        }
        .frame(width: 384)
        .background(Theme.panel)
    }

    private func fb<T>(_ kp: WritableKeyPath<Fader, T>) -> Binding<T> {
        Binding(get: { bank.faders[selected][keyPath: kp] }, set: { bank.faders[selected][keyPath: kp] = $0 })
    }

    @ViewBuilder private var faderEditor: some View {
        let f = bank.faders[selected]
        VStack(alignment: .leading, spacing: 12) {
            Text("Fader \(selected + 1)").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            if !bank.loaded { Hint(text: "Not loaded from the 16n yet: these are defaults. Use “Load” first.") }
            NoteField(text: notes.binding(NotesStore.fader(selected)))

            Card(title: "Curve", accent: Theme.function) {
                CurvePreview(curve: f.curve, color: f.curve == 64 ? Theme.cv : Theme.function)
                    .frame(height: 120)
                HStack {
                    Text(Curve.name(f.curve)).font(Theme.font(12, .semibold, .mono)).foregroundStyle(Theme.ink)
                    Spacer()
                    NumberField(value: fb(\.curve), range: 0...127)
                }
                Slider(value: Binding(get: { Double(f.curve) }, set: { bank.faders[selected].curve = Int($0.rounded()) }), in: 0...127)
                HStack(spacing: 6) {
                    ForEach(Curve.presets, id: \.1) { p in
                        Button(p.0) { bank.faders[selected].curve = p.1 }.controlSize(.small)
                    }
                }
                Button("Use this curve on all 16 faders") {
                    for k in 0..<16 { bank.faders[k].curve = f.curve }
                }
                .controlSize(.small)
                Hint(text: bank.curvesSupported || !bank.loaded
                     ? "Applied inside the 16n to USB, TRS MIDI and I2C. The CV jacks stay linear (they are wired straight to the faders)."
                     : "⚠︎ This 16n runs firmware without curves: curves are not uploaded.")
            }

            Card(title: "USB MIDI", accent: Theme.cv) {
                row("Channel") { NumberField(value: usbChannel, range: 1...16) }
                row("CC") { NumberField(value: usbCC, range: 0...127) }
                Toggle("14-bit (LSB on CC + 32)", isOn: fb(\.usb_14bit)).toggleStyle(.switch)
                    .disabled(!bank.hiresSupported && bank.loaded)
                HStack(spacing: 6) {
                    Button("Channel \(f.usb_channel) on all") { for k in 0..<16 { setUSB(k, ch: f.usb_channel, cc: bank.faders[k].usb_cc) } }
                    Button("CCs ascending from here") {
                        for k in 0..<16 { setUSB(k, ch: bank.faders[k].usb_channel, cc: min(127, f.usb_cc - selected + k)) }
                    }.disabled(f.usb_cc - selected < 0 || f.usb_cc - selected + 15 > 127)
                }.controlSize(.small)
            }

            Card(title: "Send to ER-301", accent: Theme.cv) {
                let port = mapping.faderPort(channel: f.usb_channel, cc: f.usb_cc)
                Toggle("Route through the i2c2midi", isOn: Binding(
                    get: { port != nil },
                    set: { on in mapping.setFaderPort(channel: f.usb_channel, cc: f.usb_cc, port: on ? mapping.firstFreePort() : nil) }))
                    .toggleStyle(.switch)
                if let port {
                    row("SC.CV port") {
                        NumberField(value: Binding(get: { port }, set: { mapping.setFaderPort(channel: f.usb_channel, cc: f.usb_cc, port: $0) }), range: 1...99)
                    }
                }
                Hint(text: "Needs the 16n on SSH (its USB MIDI goes through the MIDI hub on the SSH host to the i2c2midi). 0–127 → 0–10 V; with 14-bit only the MSB is used for now.")
            }

            Card(title: "TRS MIDI") {
                row("Channel") { NumberField(value: fb(\.trs_channel), range: 1...16) }
                row("CC") { NumberField(value: fb(\.trs_cc), range: 0...127) }
                Toggle("14-bit (LSB on CC + 32)", isOn: fb(\.trs_14bit)).toggleStyle(.switch)
                    .disabled(!bank.hiresSupported && bank.loaded)
            }
        }
    }

    /// Changing the USB channel / CC moves an existing ER-301 route along with it.
    private func setUSB(_ k: Int, ch: Int, cc: Int) {
        let old = bank.faders[k]
        if let p = mapping.faderPort(channel: old.usb_channel, cc: old.usb_cc), (old.usb_channel, old.usb_cc) != (ch, cc) {
            mapping.setFaderPort(channel: old.usb_channel, cc: old.usb_cc, port: nil)
            mapping.setFaderPort(channel: ch, cc: cc, port: p)
        }
        bank.faders[k].usb_channel = ch
        bank.faders[k].usb_cc = cc
    }
    private var usbChannel: Binding<Int> {
        Binding(get: { bank.faders[selected].usb_channel }, set: { setUSB(selected, ch: $0, cc: bank.faders[selected].usb_cc) })
    }
    private var usbCC: Binding<Int> {
        Binding(get: { bank.faders[selected].usb_cc }, set: { setUSB(selected, ch: bank.faders[selected].usb_channel, cc: $0) })
    }

    @ViewBuilder private var deviceEditor: some View {
        @Bindable var b = bank
        VStack(alignment: .leading, spacing: 12) {
            Text("16n").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            Hint(text: bank.loaded ? "\(bank.device), firmware \(bank.firmware)\(bank.curvesSupported ? " with curves" : ""), port “\(bank.port)”." : "Not loaded yet.")
            Card(title: "Device") {
                Toggle("LED on", isOn: $b.options.led_on).toggleStyle(.switch)
                Toggle("LED blinks on MIDI", isOn: $b.options.led_flash).toggleStyle(.switch)
                Toggle("Rotate 180° (fader 1 on the right)", isOn: $b.options.rotate).toggleStyle(.switch)
                Toggle("Soft MIDI thru", isOn: $b.options.midi_thru).toggleStyle(.switch)
                Toggle("I2C leader (sends to TXo / ER-301 / Ansible)", isOn: $b.options.i2c_leader).toggleStyle(.switch)
                Hint(text: "I2C leader takes effect after the 16n restarts. As follower (default) Teletype reads the faders.")
            }
            Card(title: "Calibration") {
                row("Fader min (raw)") { NumberField(value: $b.options.fader_min, range: 0...16383, width: 64) }
                row("Fader max (raw)") { NumberField(value: $b.options.fader_max, range: 0...16383, width: 64) }
                Hint(text: "Raw ADC values that count as fully down / up. Only change if faders don't reach 0 or 127.")
            }
        }
    }

    private func row<V: View>(_ title: String, @ViewBuilder _ v: () -> V) -> some View {
        HStack {
            Text(title).font(Theme.font(12, .medium, .ui)).foregroundStyle(Theme.ink)
            Spacer()
            v()
        }
    }
}
