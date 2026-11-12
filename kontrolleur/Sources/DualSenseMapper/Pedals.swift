import SwiftUI
import Observation

// MARK: - Model (DOREMiDi MPC-20, via scripts/mpc20_tool.py)

struct PedalSlot: Equatable, Codable {
    var slot: Int
    var kind: String          // cc pc note pitchbend pressure start continue stop tempo
    var channel: Int          // 1-16
    var number: Int           // CC / note / program
    var start: Int            // value at heel / released (pitch bend: 0-16383)
    var end: Int              // value at toe / pressed
    var mode: Int
    var gain: Int
    var ql: Int
    var raw: String           // the box's 16 bytes; unknown fields are written back unchanged
    var on: Bool
}

struct PedalInput: Equatable, Codable {
    var input: Int
    var type: String          // none expression switch hihat
    var type_id: Int
    var slots: [PedalSlot]
}

struct PedalsDTO: Decodable {
    let ok: Bool
    let error: String?
    let device: String?
    let firmware: String?
    let port: String?
    let inputs: [PedalInput]?
}

struct PedalInstallDTO: Decodable {
    let ok: Bool
    let error: String?
    let verified: Bool?
    let written: Int?
    let skipped: [String]?
    let mismatch: [String]?
    let inputs: [PedalInput]?
}

enum PedalKind: String, CaseIterable, Identifiable {
    case cc, note, pc, pitchbend, pressure, start, `continue`, stop
    var id: String { rawValue }
    var label: String {
        switch self {
        case .cc: "CC"
        case .note: "Note"
        case .pc: "Program"
        case .pitchbend: "Pitch bend"
        case .pressure: "Ch. pressure"
        case .start: "Start"
        case .continue: "Continue"
        case .stop: "Stop"
        }
    }
    var short: String {
        switch self {
        case .cc: "CC"
        case .note: "N"
        case .pc: "PC"
        case .pitchbend: "PB"
        case .pressure: "AT"
        case .start: "START"
        case .continue: "CONT"
        case .stop: "STOP"
        }
    }
    var hasNumber: Bool { [.cc, .note, .pc].contains(self) }
    var hasRange: Bool { [.cc, .note, .pitchbend, .pressure].contains(self) }
    var maxValue: Int { self == .pitchbend ? 16383 : 127 }

    /// What each pedal type can send (as the Pedal Config Tool offers it).
    static func choices(for type: String) -> [PedalKind] {
        switch type {
        case "switch": [.cc, .note, .pc, .start, .continue, .stop]
        case "hihat": [.cc, .note]
        default: [.cc, .note, .pitchbend, .pressure]
        }
    }
}

@MainActor @Observable
final class PedalBoard {
    var inputs: [PedalInput] = (0..<8).map { i in
        PedalInput(input: i, type: "none", type_id: 0, slots: (0..<8).map {
            PedalSlot(slot: $0, kind: "cc", channel: 1, number: 0, start: 0, end: 0, mode: 0, gain: 50, ql: 50, raw: "", on: false)
        })
    }
    var loaded = false
    var device = ""
    var firmware = ""
    var port = ""

    func apply(_ list: [PedalInput]) {
        if list.count >= 4 { inputs = list }
        loaded = true
    }

    func apply(_ d: PedalsDTO) {
        if let l = d.inputs { apply(l) }
        device = d.device ?? ""
        firmware = d.firmware ?? ""
        port = d.port ?? ""
    }

    var json: String {
        struct Out: Encodable { let inputs: [PedalInput] }
        let enc = JSONEncoder(); enc.outputFormatting = [.sortedKeys]
        return String(decoding: (try? enc.encode(Out(inputs: inputs))) ?? Data(), as: UTF8.self)
    }

    var problems: [String] {
        var p: [String] = []
        for inp in inputs where inp.type_id != 0 {
            for s in inp.slots where s.on {
                let k = PedalKind(rawValue: s.kind) ?? .cc
                if !(1...16).contains(s.channel) { p.append("\(PedalBoard.name(inp.input)) message \(s.slot + 1): channel must be 1–16.") }
                if k.hasRange && s.start == s.end { p.append("\(PedalBoard.name(inp.input)) message \(s.slot + 1): start and end are the same value.") }
            }
        }
        return p
    }

    static func name(_ i: Int) -> String { i < 4 ? "EXP\(i + 1)" : "EXP\(i - 3) ring" }

    /// Pedal messages on channel 1 follow the DualSense instrument channel in the Pi hub.
    static func hubChannel(_ ch: Int, mapping: Mapping) -> Int { ch == 1 ? mapping.channel : ch }
}

// MARK: - Running the helper (Pi over ssh, or this Mac)

struct PedalClient {
    var onPi: Bool
    var pi: PiClient
    var python: String
    var repo: String

    enum Failure: LocalizedError {
        case msg(String)
        var errorDescription: String? { if case .msg(let s) = self { return s }; return nil }
    }

    private func runLocal(_ arg: String, input: String?) async throws -> String {
        guard let script = Bundle.main.path(forResource: "mpc20_tool", ofType: "py") else {
            throw Failure.msg("mpc20_tool.py missing in the app bundle")
        }
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
            throw Failure.msg(text.isEmpty ? "no answer from mpc20_tool" : String(text.suffix(300)))
        }
        return try JSONDecoder().decode(T.self, from: data)
    }

    func export() async throws -> PedalsDTO {
        let text = onPi ? try await pi.runRemote("python3 \(repo)/scripts/mpc20_tool.py export")
                        : try await runLocal("export", input: nil)
        return try decode(text)
    }

    func install(_ json: String) async throws -> PedalInstallDTO {
        let text = onPi ? try await pi.runRemote("python3 \(repo)/scripts/mpc20_tool.py install", input: json)
                        : try await runLocal("install", input: json)
        return try decode(text)
    }
}

// MARK: - Drawing

@MainActor
struct PedalCanvas: View {
    @Environment(PedalBoard.self) private var board
    @Binding var selected: Int

    var body: some View {
        ZoomStage(size: CGSize(width: 1100, height: 640)) {
            ZStack(alignment: .topLeading) {
                BlueprintGrid()
                Text(board.loaded ? "DOREMiDi \(board.device) · firmware \(board.firmware)" : "DOREMiDi MPC-20 · not loaded")
                    .font(Theme.font(11, .heavy, .mono)).tracking(1.2)
                    .foregroundStyle(Theme.dim)
                    .position(x: 550, y: 40)
                box
                ForEach(0..<4, id: \.self) { i in
                    cable(i)
                    PedalTile(index: i, selected: selected == i)
                        .position(x: jackX(i), y: 430)
                        .onTapGesture { selected = i }
                }
            }
        }
    }

    private func jackX(_ i: Int) -> CGFloat { 550 + (CGFloat(i) - 1.5) * 230 }

    private var box: some View {
        ZStack {
            RoundedRectangle(cornerRadius: 18)
                .fill(LinearGradient(colors: [Theme.body.opacity(0.12), Theme.body.opacity(0.04)], startPoint: .top, endPoint: .bottom))
                .overlay(RoundedRectangle(cornerRadius: 18).stroke(Theme.body.opacity(0.6), lineWidth: 1.6))
                .frame(width: 900, height: 120)
            HStack(spacing: 10) {
                Text("MPC-20").font(Theme.font(22, .heavy, .title)).foregroundStyle(Theme.ink)
                Text("pedal → MIDI").font(Theme.font(11, .semibold, .mono)).foregroundStyle(Theme.dim)
            }
            .offset(y: -28)
            ForEach(0..<4, id: \.self) { i in
                VStack(spacing: 5) {
                    ZStack {
                        Circle().fill(Theme.bg).frame(width: 26, height: 26)
                        Circle().stroke(selected == i ? Theme.ink : Theme.body.opacity(0.8), lineWidth: 2).frame(width: 26, height: 26)
                        Circle().fill(Theme.body.opacity(0.5)).frame(width: 8, height: 8)
                    }
                    Text("EXP\(i + 1)").font(Theme.font(10, .heavy, .mono))
                        .foregroundStyle(selected == i ? Theme.ink : Theme.dim)
                }
                .offset(x: jackX(i) - 550, y: 22)
                .onTapGesture { selected = i }
            }
        }
        .position(x: 550, y: 150)
    }

    private func cable(_ i: Int) -> some View {
        let used = board.inputs[i].type_id != 0
        return Path { p in
            p.move(to: CGPoint(x: jackX(i), y: 211))
            p.addCurve(to: CGPoint(x: jackX(i), y: 318),
                       control1: CGPoint(x: jackX(i) + 30, y: 245), control2: CGPoint(x: jackX(i) - 30, y: 280))
        }
        .stroke(used ? (selected == i ? Theme.ink.opacity(0.8) : Theme.cv.opacity(0.7)) : Theme.body.opacity(0.18),
                style: StrokeStyle(lineWidth: used ? 3 : 1.4, lineCap: .round, dash: used ? [] : [4, 5]))
    }
}

@MainActor
struct PedalTile: View {
    @Environment(PedalBoard.self) private var board
    @Environment(Mapping.self) private var mapping
    @Environment(NotesStore.self) private var notes
    let index: Int
    let selected: Bool

    var body: some View {
        let inp = board.inputs[index]
        let active = inp.slots.filter(\.on)
        let col: Color = inp.type == "switch" ? Theme.trig : (inp.type == "hihat" ? Theme.function : Theme.cv)
        VStack(spacing: 10) {
            PedalIcon(type: inp.type, color: col, selected: selected)
                .frame(width: 120, height: 92)
            Text(inp.type_id == 0 ? "nothing plugged in" : typeName(inp.type))
                .font(Theme.font(11, .bold, .mono))
                .foregroundStyle(inp.type_id == 0 ? Theme.dim : Theme.ink)
            VStack(spacing: 3) {
                ForEach(active.prefix(5), id: \.slot) { s in
                    let k = PedalKind(rawValue: s.kind) ?? .cc
                    let port = k == .cc ? mapping.faderPort(channel: PedalBoard.hubChannel(s.channel, mapping: mapping), cc: s.number) : nil
                    HStack(spacing: 5) {
                        Text("ch\(s.channel)").foregroundStyle(Theme.dim)
                        Text(k.hasNumber ? "\(k.short)\(s.number)" : k.short).foregroundStyle(Theme.ink)
                        if let port {
                            Text("CV \(port)").foregroundStyle(Theme.cv)
                                .padding(.horizontal, 4).background(Capsule().fill(Theme.cv.opacity(0.14)))
                        }
                        if let n = notes.note(NotesStore.pedal(index, s.slot)) {
                            NoteBadge(text: n, size: 9).frame(maxWidth: 80, alignment: .leading)
                        }
                    }
                    .font(Theme.font(10, .semibold, .mono))
                }
                if active.count > 5 {
                    Text("+\(active.count - 5) more").font(Theme.font(10, .regular, .mono)).foregroundStyle(Theme.dim)
                }
                if inp.type_id != 0 && active.isEmpty {
                    Text("no messages").font(Theme.font(10, .regular, .mono)).foregroundStyle(Theme.dim)
                }
            }
            .frame(height: 110, alignment: .top)
        }
        .padding(12)
        .frame(width: 190)
        .background(RoundedRectangle(cornerRadius: 12).fill(selected ? col.opacity(0.10) : Color.clear))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(selected ? Theme.ink.opacity(0.7) : Color.clear, lineWidth: 1.2))
        .shadow(color: selected ? col.opacity(0.5) : .clear, radius: 10)
        .contentShape(Rectangle())
    }

    private func typeName(_ t: String) -> String {
        switch t {
        case "expression": "Expression pedal"
        case "switch": "Switch pedal"
        case "hihat": "Hi-hat pedal"
        default: t
        }
    }
}

struct PedalIcon: View {
    let type: String
    let color: Color
    let selected: Bool

    var body: some View {
        Canvas { ctx, size in
            let w = size.width, h = size.height
            let stroke = selected ? Theme.ink : color
            switch type {
            case "expression", "hihat":
                // base
                var base = Path()
                base.addRoundedRect(in: CGRect(x: w * 0.08, y: h * 0.72, width: w * 0.84, height: h * 0.2), cornerSize: CGSize(width: 5, height: 5))
                ctx.fill(base, with: .color(color.opacity(0.18)))
                ctx.stroke(base, with: .color(stroke), lineWidth: 1.5)
                // rocking treadle
                var tread = Path()
                tread.move(to: CGPoint(x: w * 0.12, y: h * 0.70))
                tread.addLine(to: CGPoint(x: w * 0.86, y: h * 0.30))
                tread.addLine(to: CGPoint(x: w * 0.92, y: h * 0.40))
                tread.addLine(to: CGPoint(x: w * 0.18, y: h * 0.78))
                tread.closeSubpath()
                ctx.fill(tread, with: .color(color.opacity(0.30)))
                ctx.stroke(tread, with: .color(stroke), lineWidth: 1.5)
                var pivot = Path(); pivot.addEllipse(in: CGRect(x: w * 0.47, y: h * 0.52, width: 9, height: 9))
                ctx.fill(pivot, with: .color(stroke))
                if type == "hihat" {
                    var c = Path(); c.addEllipse(in: CGRect(x: w * 0.62, y: h * 0.02, width: w * 0.3, height: h * 0.12))
                    ctx.stroke(c, with: .color(stroke), lineWidth: 1.3)
                }
            case "switch":
                var b = Path()
                b.addRoundedRect(in: CGRect(x: w * 0.2, y: h * 0.30, width: w * 0.6, height: h * 0.58), cornerSize: CGSize(width: 8, height: 8))
                ctx.fill(b, with: .color(color.opacity(0.18)))
                ctx.stroke(b, with: .color(stroke), lineWidth: 1.5)
                var k = Path(); k.addEllipse(in: CGRect(x: w * 0.38, y: h * 0.42, width: w * 0.24, height: w * 0.24))
                ctx.fill(k, with: .color(color.opacity(0.4)))
                ctx.stroke(k, with: .color(stroke), lineWidth: 1.5)
            default:
                var e = Path()
                e.addRoundedRect(in: CGRect(x: w * 0.15, y: h * 0.3, width: w * 0.7, height: h * 0.55), cornerSize: CGSize(width: 8, height: 8))
                ctx.stroke(e, with: .color(Theme.body.opacity(0.35)), style: StrokeStyle(lineWidth: 1.2, dash: [4, 4]))
            }
        }
    }
}

// MARK: - Inspector

@MainActor
struct PedalPanel: View {
    @Environment(PedalBoard.self) private var board
    @Environment(Mapping.self) private var mapping
    @Binding var selected: Int
    @State private var tab = "Pedal"

    var body: some View {
        VStack(spacing: 0) {
            Picker("", selection: $tab) { ForEach(["Pedal", "Device"], id: \.self) { Text($0).tag($0) } }
                .pickerStyle(.segmented).labelsHidden().padding(14)
            Divider().overlay(Theme.line)
            ScrollView {
                Group { if tab == "Pedal" { editor } else { device } }
                    .padding(14)
            }
        }
        .frame(width: 384)
        .background(Theme.panel)
    }

    @ViewBuilder private var device: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("MPC-20").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            Hint(text: board.loaded ? "DOREMiDi \(board.device), firmware \(board.firmware), connected via \(board.port)." : "Not loaded yet. Use “Load”.")
            Card(title: "Jacks") {
                ForEach(0..<4, id: \.self) { i in
                    let inp = board.inputs[i]
                    HStack {
                        Text("EXP\(i + 1)").font(Theme.font(12, .bold, .mono)).foregroundStyle(Theme.ink)
                            .frame(width: 52, alignment: .leading)
                        Text(inp.type_id == 0 ? "nothing plugged in" : inp.type)
                            .font(Theme.font(12, .regular, .ui)).foregroundStyle(inp.type_id == 0 ? Theme.dim : Theme.ink)
                        Spacer()
                        Text("\(inp.slots.filter(\.on).count) messages").font(Theme.font(11, .regular, .mono)).foregroundStyle(Theme.dim)
                    }
                }
                Hint(text: "The box detects the pedal type by itself. Settings are stored per jack and pedal type, and kept when it's unplugged.")
            }
            Card(title: "Backup") {
                Hint(text: "Save Configuration (⌘⇧S, Configs menu) includes the pedal settings. The SSH host also has a raw backup: scripts/mpc20_backup_20261111.json (mpc20_tool.py restore).")
            }
        }
    }

    @ViewBuilder private var editor: some View {
        let inp = board.inputs[selected]
        VStack(alignment: .leading, spacing: 12) {
            Text("EXP\(selected + 1)").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            if !board.loaded {
                Hint(text: "Not loaded from the MPC-20 yet. Use “Load” first.")
            } else if inp.type_id == 0 {
                Hint(text: "Nothing is plugged into this jack. The MPC-20 recognises the pedal type by itself: plug the pedal in, then press “Load”.")
            } else {
                Hint(text: "\(typeLine(inp.type)) All messages that are switched on are sent together.")
                ForEach(0..<8, id: \.self) { s in
                    if inp.slots[s].on || s == firstOff(inp) {
                        SlotCard(input: selected, slot: s)
                    }
                }
                Hint(text: board.inputs[selected].type == "switch"
                     ? "Start = released, end = pressed. Curve, gain and trigger settings stay as they are on the box."
                     : "Start = heel, end = toe (swap them to invert). Curve, gain and trigger settings stay as they are on the box.")
            }
        }
    }

    private func firstOff(_ inp: PedalInput) -> Int? { inp.slots.firstIndex { !$0.on } }

    private func typeLine(_ t: String) -> String {
        switch t {
        case "expression": "Expression pedal (detected by the box)."
        case "switch": "Switch pedal (detected by the box)."
        case "hihat": "Hi-hat pedal (detected by the box)."
        default: ""
        }
    }
}

@MainActor
struct SlotCard: View {
    @Environment(PedalBoard.self) private var board
    @Environment(Mapping.self) private var mapping
    @Environment(NotesStore.self) private var notes
    let input: Int
    let slot: Int

    private var s: PedalSlot { board.inputs[input].slots[slot] }
    private func b<T>(_ kp: WritableKeyPath<PedalSlot, T>) -> Binding<T> {
        Binding(get: { board.inputs[input].slots[slot][keyPath: kp] }, set: { board.inputs[input].slots[slot][keyPath: kp] = $0 })
    }

    var body: some View {
        let kind = PedalKind(rawValue: s.kind) ?? .cc
        let type = board.inputs[input].type
        if !s.on {
            Button {
                var n = s
                n.on = true
                n.kind = "cc"
                n.channel = board.inputs[input].slots.first(where: \.on)?.channel ?? 1
                n.number = nextFreeCC()
                n.start = 0; n.end = 127
                board.inputs[input].slots[slot] = n
            } label: {
                Label("Add a message", systemImage: "plus.circle")
            }
            .controlSize(.small)
        } else {
            Card(title: "Message \(slot + 1)", accent: Theme.cv) {
                HStack {
                    Picker("", selection: Binding(get: { kind }, set: { setKind($0) })) {
                        ForEach(PedalKind.choices(for: type)) { Text($0.label).tag($0) }
                    }
                    .labelsHidden().frame(width: 140)
                    Spacer()
                    Button(role: .destructive) { remove() } label: { Image(systemName: "trash") }
                        .buttonStyle(.borderless).help("Switch this message off")
                }
                NoteField(text: notes.binding(NotesStore.pedal(input, slot)))
                row("Channel") { NumberField(value: channelBinding, range: 1...16) }
                if kind.hasNumber {
                    row(kind == .note ? "Note" : (kind == .pc ? "Program" : "CC")) { NumberField(value: numberBinding, range: 0...127) }
                }
                if kind.hasRange {
                    row(type == "switch" ? "Released → pressed" : "Heel → toe") {
                        HStack(spacing: 4) {
                            NumberField(value: b(\.start), range: 0...kind.maxValue, width: kind == .pitchbend ? 60 : 44)
                            Image(systemName: "arrow.right").foregroundStyle(Theme.dim).imageScale(.small)
                            NumberField(value: b(\.end), range: 0...kind.maxValue, width: kind == .pitchbend ? 60 : 44)
                        }
                    }
                }
                if type == "switch" {
                    row("Mode") {
                        Picker("", selection: b(\.mode)) {
                            Text("Momentary").tag(0)
                            Text("Toggle").tag(1)
                            if s.mode > 1 { Text("Other (\(s.mode))").tag(s.mode) }
                        }
                        .labelsHidden().frame(width: 130)
                    }
                }
                if kind == .cc {
                    let ch = PedalBoard.hubChannel(s.channel, mapping: mapping)
                    let port = mapping.faderPort(channel: ch, cc: s.number)
                    Toggle("Send to the ER-301", isOn: Binding(
                        get: { port != nil },
                        set: { on in mapping.setFaderPort(channel: ch, cc: s.number, port: on ? mapping.firstFreePort() : nil) }))
                        .toggleStyle(.switch)
                    if let port {
                        row("SC.CV port") {
                            NumberField(value: Binding(get: { port }, set: { mapping.setFaderPort(channel: ch, cc: s.number, port: $0) }), range: 1...99)
                        }
                        Hint(text: s.channel == 1
                             ? "The MIDI hub moves pedal messages on channel 1 to the DualSense channel (\(mapping.channel)); the route follows that."
                             : "Routed on channel \(s.channel).")
                    }
                }
            }
        }
    }

    private var channelBinding: Binding<Int> {
        Binding(get: { s.channel }, set: { move(channel: $0, number: s.number) })
    }
    private var numberBinding: Binding<Int> {
        Binding(get: { s.number }, set: { move(channel: s.channel, number: $0) })
    }

    /// Changing channel / CC moves an existing ER-301 route along with it.
    private func move(channel: Int, number: Int) {
        if s.kind == "cc" {
            let oldCh = PedalBoard.hubChannel(s.channel, mapping: mapping)
            let newCh = PedalBoard.hubChannel(channel, mapping: mapping)
            if let p = mapping.faderPort(channel: oldCh, cc: s.number), (oldCh, s.number) != (newCh, number) {
                mapping.setFaderPort(channel: oldCh, cc: s.number, port: nil)
                mapping.setFaderPort(channel: newCh, cc: number, port: p)
            }
        }
        board.inputs[input].slots[slot].channel = channel
        board.inputs[input].slots[slot].number = number
    }

    private func setKind(_ k: PedalKind) {
        if s.kind == "cc", k != .cc {
            mapping.setFaderPort(channel: PedalBoard.hubChannel(s.channel, mapping: mapping), cc: s.number, port: nil)
        }
        var n = s
        n.kind = k.rawValue
        if k == .pitchbend { n.start = 0; n.end = 16383 }
        else { n.start = min(n.start, 127); n.end = min(n.end, 127); if n.start == n.end { n.end = 127 } }
        board.inputs[input].slots[slot] = n
    }

    private func remove() {
        if s.kind == "cc" {
            mapping.setFaderPort(channel: PedalBoard.hubChannel(s.channel, mapping: mapping), cc: s.number, port: nil)
        }
        board.inputs[input].slots[slot].on = false
    }

    private func nextFreeCC() -> Int {
        let used = Set(board.inputs.flatMap(\.slots).filter { $0.on && $0.kind == "cc" }.map(\.number))
        let last = board.inputs[input].slots.filter { $0.on && $0.kind == "cc" }.map(\.number).max() ?? 101
        var c = last + 1
        while used.contains(c) && c < 127 { c += 1 }
        return min(c, 127)
    }

    private func row<V: View>(_ title: String, @ViewBuilder _ v: () -> V) -> some View {
        HStack {
            Text(title).font(Theme.font(12, .medium, .ui)).foregroundStyle(Theme.ink)
            Spacer()
            v()
        }
    }
}
