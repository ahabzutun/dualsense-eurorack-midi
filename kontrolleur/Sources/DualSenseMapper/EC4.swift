import SwiftUI
import Observation

// MARK: - Model (Faderfox EC4, via scripts/ec4_tool.py)

struct EC4Push: Equatable, Codable {
    var type: String
    var channel: Int
    var number: Int
    var mode: String
    var display: Bool
    var lower: Int
    var upper: Int
    var link: Bool
}

struct EC4Encoder: Equatable, Codable {
    var name: String
    var type: String
    var channel: Int
    var number: Int
    var number_h: Int
    var lower: Int
    var upper: Int
    var mode: String
    var scale: String
    var link: Bool
    var push: EC4Push
}

struct EC4Group: Equatable, Codable {
    var name: String
    var encoders: [EC4Encoder]
}

struct EC4Setup: Equatable, Codable {
    var name: String
    var groups: [EC4Group]
}

struct EC4DTO: Codable {
    let ok: Bool
    let error: String?
    let version: Double?
    let port: String?
    let mem: String?
    let setups: [EC4Setup]?
}

struct EC4SendDTO: Decodable {
    let ok: Bool
    let error: String?
    let bytes: Int?
    let changed_bytes: Int?
}

enum EC4Names {
    static let types: [(String, String)] = [
        ("cc_rel1", "CC relative 1"), ("cc_rel2", "CC relative 2"), ("cc", "CC absolute"), ("pc", "Program change"),
        ("cc14", "CC 14-bit"), ("pitchbend", "Pitch bend"), ("aftertouch", "Aftertouch"), ("note", "Note"), ("nrpn", "NRPN")]
    static let modes: [(String, String)] = [
        ("div8", "÷ 8 (fine)"), ("div4", "÷ 4"), ("div2", "÷ 2"), ("none", "no acceleration"), ("low", "low acceleration"),
        ("middle", "middle acceleration"), ("max", "max acceleration"), ("step2", "large step 2"), ("step4", "large step 4"),
        ("step6", "large step 6")]
    static let scales: [(String, String)] = [
        ("off", "display off"), ("0-127", "0 … 127"), ("0-100", "0 … 100"), ("0-1000", "0 … 1000"), ("-63..63", "−63 … +63"),
        ("-50..50", "−50 … +50"), ("-500..500", "−500 … +500"), ("on/off", "ON / OFF"), ("0-9999", "0 … 9999")]
    static let pushTypes: [(String, String)] = [
        ("off", "off"), ("note", "Note"), ("cc", "CC"), ("pc", "Program change"), ("pitchbend", "Pitch bend"),
        ("aftertouch", "Aftertouch"), ("group", "select group"), ("setup", "select setup"), ("acc0", "acceleration 0"),
        ("acc3", "acceleration 3"), ("lsp6", "large step 6"), ("min", "set to min"), ("max", "set to max")]
    static func short(_ t: String) -> String {
        switch t {
        case "cc_rel1", "cc_rel2": "REL"
        case "cc": "CC"
        case "pc": "PC"
        case "cc14": "CC14"
        case "pitchbend": "PB"
        case "aftertouch": "AT"
        case "note": "N"
        case "nrpn": "NRPN"
        default: t
        }
    }
    /// 14-bit ranges as in the Faderfox editor (CC 14-bit / PB / NRPN with a wide display).
    static func highres(_ e: EC4Encoder) -> Bool {
        ["cc14", "pitchbend", "nrpn"].contains(e.type) && ["off", "0-1000", "-500..500", "0-9999"].contains(e.scale)
    }
}

@MainActor @Observable
final class EC4Board {
    var setups: [EC4Setup] = []
    var mem: String = ""
    var version: Double = 0
    var loaded: Bool { !mem.isEmpty && setups.count == 16 }
    var receivedAt: Date? = nil

    func apply(_ d: EC4DTO) {
        guard let s = d.setups, s.count == 16, let m = d.mem else { return }
        setups = s
        mem = m
        version = d.version ?? 0
    }

    var json: String {
        struct Out: Encodable { let mem: String; let setups: [EC4Setup] }
        let enc = JSONEncoder(); enc.outputFormatting = [.sortedKeys]
        return String(decoding: (try? enc.encode(Out(mem: mem, setups: setups))) ?? Data(), as: UTF8.self)
    }

    /// Only the editable part, for "unsaved changes".
    var fingerprint: String {
        let enc = JSONEncoder(); enc.outputFormatting = [.sortedKeys]
        return String(decoding: (try? enc.encode(setups)) ?? Data(), as: UTF8.self)
    }

    // The last dump is kept on this Mac, so the tab shows something without a new transfer.
    static var cacheURL: URL {
        let dir = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("DualSense Mapper", isDirectory: true)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir.appendingPathComponent("ec4-last.json")
    }

    func saveCache() {
        let dto = EC4DTO(ok: true, error: nil, version: version, port: nil, mem: mem, setups: setups)
        if let d = try? JSONEncoder().encode(dto) { try? d.write(to: EC4Board.cacheURL) }
    }

    func loadCache() -> Bool {
        guard let d = try? Data(contentsOf: EC4Board.cacheURL), let dto = try? JSONDecoder().decode(EC4DTO.self, from: d) else { return false }
        apply(dto)
        receivedAt = (try? FileManager.default.attributesOfItem(atPath: EC4Board.cacheURL.path)[.modificationDate]) as? Date
        return loaded
    }

    var problems: [String] {
        var p: [String] = []
        for (s, su) in setups.enumerated() {
            for (g, gr) in su.groups.enumerated() {
                for (e, en) in gr.encoders.enumerated() where en.type == "cc14" && en.number > 31 {
                    p.append("Setup \(s + 1) group \(g + 1) encoder \(e + 1): 14-bit CC needs a CC number 0–31.")
                }
            }
        }
        return p
    }
}

// MARK: - Running the helper

struct EC4Client {
    var onPi: Bool
    var pi: PiClient
    var python: String
    var repo: String

    enum Failure: LocalizedError {
        case msg(String)
        var errorDescription: String? { if case .msg(let s) = self { return s }; return nil }
    }

    private func runLocal(_ args: [String], input: String?) async throws -> String {
        guard let script = Bundle.main.path(forResource: "ec4_tool", ofType: "py") else {
            throw Failure.msg("ec4_tool.py missing in the app bundle")
        }
        let python = python
        return try await withCheckedThrowingContinuation { cont in
            DispatchQueue.global(qos: .userInitiated).async {
                let p = Process()
                p.executableURL = URL(fileURLWithPath: (python as NSString).expandingTildeInPath)
                p.arguments = [script] + args
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
            throw Failure.msg(text.isEmpty ? "no answer from ec4_tool" : String(text.suffix(300)))
        }
        return try JSONDecoder().decode(T.self, from: data)
    }

    func receive(seconds: Int = 120) async throws -> EC4DTO {
        let text = onPi ? try await pi.runRemote("python3 \(repo)/scripts/ec4_tool.py receive \(seconds)")
                        : try await runLocal(["receive", String(seconds)], input: nil)
        return try decode(text)
    }

    func send(_ json: String) async throws -> EC4SendDTO {
        let text = onPi ? try await pi.runRemote("python3 \(repo)/scripts/ec4_tool.py send", input: json)
                        : try await runLocal(["send"], input: json)
        return try decode(text)
    }
}

// MARK: - Drawing

@MainActor
struct EC4Canvas: View {
    @Environment(EC4Board.self) private var board
    @Environment(NotesStore.self) private var notes
    @Binding var setup: Int
    @Binding var group: Int
    @Binding var selected: Int

    var body: some View {
        // A flow layout: everything sizes itself to its text, the stage grows with it
        ZoomStage(size: CGSize(width: 1100, height: 640), grows: true) {
            if board.loaded {
                device
            } else {
                VStack(spacing: 10) {
                    Text("Faderfox EC4").font(Theme.font(22, .heavy, .title)).foregroundStyle(Theme.ink)
                    Text("No setups received yet.\nPress “Receive”, then on the EC4: Func → Setup → Send → “Send all setups” (hold the knob).")
                        .multilineTextAlignment(.center)
                        .font(Theme.font(13, .regular, .mono)).foregroundStyle(Theme.dim)
                }
            }
        }
    }

    private var device: some View {
        let gr = board.setups[setup].groups[group]
        return VStack(spacing: 20) {
            Text("EC4 · firmware data \(String(format: "%.1f", board.version))\(board.receivedAt.map { " · received \($0.formatted(date: .abbreviated, time: .shortened))" } ?? "")")
                .font(Theme.font(11, .heavy, .mono)).tracking(1.2)
                .foregroundStyle(Theme.dim)
            // setup + group strip
            Grid(horizontalSpacing: 3, verticalSpacing: 6) {
                strip(title: "SETUP", names: board.setups.map(\.name), sel: setup) { setup = $0; group = 0 }
                strip(title: "GROUP", names: board.setups[setup].groups.map(\.name), sel: group) { group = $0 }
            }
            // body with 4x4 encoders; the body outline wraps the tiles
            Grid(horizontalSpacing: 6, verticalSpacing: 6) {
                ForEach(0..<4, id: \.self) { row in
                    GridRow {
                        ForEach(0..<4, id: \.self) { col in
                            let i = row * 4 + col
                            EncoderTile(enc: gr.encoders[i], index: i, selected: selected == i,
                                        note: notes.note(NotesStore.ec4(setup, group, i)))
                                .onTapGesture { selected = i }
                        }
                    }
                }
            }
            .padding(.horizontal, 26).padding(.vertical, 22)
            .background(
                RoundedRectangle(cornerRadius: 22)
                    .fill(LinearGradient(colors: [Theme.body.opacity(0.10), Theme.body.opacity(0.03)], startPoint: .top, endPoint: .bottom))
                    .overlay(RoundedRectangle(cornerRadius: 22).stroke(Theme.body.opacity(0.55), lineWidth: 1.6))
            )
        }
    }

    private func strip(title: String, names: [String], sel: Int, pick: @escaping (Int) -> Void) -> some View {
        GridRow {
            Text(title).font(Theme.font(9, .heavy, .mono)).foregroundStyle(Theme.dim)
                .fixedSize()
                .gridColumnAlignment(.leading)
                .padding(.trailing, 6)
            ForEach(0..<16, id: \.self) { i in
                let n = names[i].trimmingCharacters(in: .whitespaces)
                Text(n.isEmpty ? "\(i + 1)" : n)
                    .font(Theme.font(10, .bold, .mono))
                    .lineLimit(1)
                    .foregroundStyle(i == sel ? Theme.bg : (n.isEmpty ? Theme.dim : Theme.ink))
                    .padding(.horizontal, 5).padding(.vertical, 4)
                    .frame(minWidth: 52, minHeight: 22)
                    .background(RoundedRectangle(cornerRadius: 5).fill(i == sel ? Theme.cv : Theme.body.opacity(0.10)))
                    .contentShape(Rectangle())
                    .onTapGesture { pick(i) }
            }
        }
    }
}

@MainActor
struct EncoderTile: View {
    @Environment(Mapping.self) private var mapping
    let enc: EC4Encoder
    let index: Int
    let selected: Bool
    var note: String? = nil

    var body: some View {
        let port = enc.type == "cc" ? mapping.faderPort(channel: enc.channel, cc: enc.number) : nil
        let col: Color = enc.type.hasPrefix("cc_rel") ? Theme.function : (enc.type == "note" || enc.type == "pc" ? Theme.trig : Theme.cv)
        let g = Theme.grow
        VStack(spacing: 6) {
            Text(enc.name.trimmingCharacters(in: .whitespaces).isEmpty ? "—" : enc.name)
                .font(Theme.font(12, .heavy, .mono))
                .lineLimit(1).minimumScaleFactor(0.7)
                .foregroundStyle(Color(red: 0.55, green: 0.95, blue: 0.75))
                .padding(.vertical, 2)
                .frame(width: 96 * g)
                .background(RoundedRectangle(cornerRadius: 3).fill(Color.black.opacity(0.65)))
            ZStack {
                Circle().fill(Theme.bg).frame(width: 52, height: 52)
                Circle().stroke(selected ? Theme.ink : col.opacity(0.8), lineWidth: 2).frame(width: 52, height: 52)
                Rectangle().fill(selected ? Theme.ink : col).frame(width: 2.5, height: 16).offset(y: -15)
            }
            HStack(spacing: 4) {
                Text("ch\(enc.channel)").foregroundStyle(Theme.dim)
                Text(enc.type == "pitchbend" || enc.type == "aftertouch" ? EC4Names.short(enc.type) : "\(EC4Names.short(enc.type))\(enc.number)")
                    .foregroundStyle(Theme.ink)
            }
            .font(Theme.font(10, .semibold, .mono))
            .lineLimit(1).minimumScaleFactor(0.7)
            let pushText: String? = enc.push.type == "off" ? nil : "push: \(enc.push.type)"
            if port != nil || note != nil || pushText != nil {
            HStack(spacing: 5) {
                if port != nil || note == nil {
                    Text(port.map { "CV \($0)" } ?? pushText ?? "")
                        .font(Theme.font(9, .bold, .mono))
                        .lineLimit(1)
                        .foregroundStyle(port == nil ? Theme.dim : Theme.cv)
                }
                if let note { NoteBadge(text: note, size: 9).frame(maxWidth: (port == nil ? 126 : 80) * g) }
            }
            }
        }
        // width grows with the text size; height comes from the content, so the
        // selection outline always wraps all four lines
        .padding(.vertical, 8)
        .frame(width: 140 * g)
        .frame(maxHeight: .infinity, alignment: .top)   // all tiles in a row get the same height
        .background(RoundedRectangle(cornerRadius: 10).fill(selected ? col.opacity(0.12) : Color.clear))
        .overlay(RoundedRectangle(cornerRadius: 10).stroke(selected ? Theme.ink.opacity(0.7) : Color.clear, lineWidth: 1.2))
        .contentShape(Rectangle())
    }
}

// MARK: - Inspector

enum EC4Tab: String, CaseIterable, Identifiable {
    case encoder = "Encoder", push = "Push", names = "Names"
    var id: String { rawValue }
}

@MainActor
struct EC4Panel: View {
    @Environment(EC4Board.self) private var board
    @Environment(Mapping.self) private var mapping
    @Environment(NotesStore.self) private var notes
    let setup: Int
    let group: Int
    @Binding var selected: Int
    @State private var tab: EC4Tab = .encoder

    var body: some View {
        VStack(spacing: 0) {
            Picker("", selection: $tab) { ForEach(EC4Tab.allCases) { Text($0.rawValue).tag($0) } }
                .pickerStyle(.segmented).labelsHidden().padding(14)
            Divider().overlay(Theme.line)
            ScrollView {
                Group {
                    if !board.loaded {
                        Hint(text: "Receive the setups from the EC4 first.")
                    } else {
                        switch tab {
                        case .encoder: encoderEditor
                        case .push: pushEditor
                        case .names: namesEditor
                        }
                    }
                }
                .padding(14)
            }
        }
        .frame(width: 384)
        .background(Theme.panel)
    }

    private var enc: EC4Encoder { board.setups[setup].groups[group].encoders[selected] }
    private func eb<T>(_ kp: WritableKeyPath<EC4Encoder, T>) -> Binding<T> {
        Binding(get: { board.setups[setup].groups[group].encoders[selected][keyPath: kp] },
                set: { board.setups[setup].groups[group].encoders[selected][keyPath: kp] = $0 })
    }

    @ViewBuilder private var encoderEditor: some View {
        let e = enc
        VStack(alignment: .leading, spacing: 12) {
            Text("Encoder \(selected + 1)").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            Hint(text: "Setup \(setup + 1) “\(board.setups[setup].name)” · group \(group + 1) “\(board.setups[setup].groups[group].name)”")
            NoteField(text: notes.binding(NotesStore.ec4(setup, group, selected)))
            Card(title: "Turn", accent: Theme.cv) {
                row("Name") { NameField(text: eb(\.name)) }
                row("Type") {
                    Picker("", selection: eb(\.type)) { ForEach(EC4Names.types, id: \.0) { Text($0.1).tag($0.0) } }
                        .labelsHidden().frame(width: 170)
                }
                row("Channel") { NumberField(value: channelBinding, range: 1...16) }
                if e.type == "nrpn" {
                    row("NRPN MSB / LSB") {
                        HStack(spacing: 4) {
                            NumberField(value: eb(\.number_h), range: 0...127)
                            NumberField(value: numberBinding, range: 0...127)
                        }
                    }
                } else if !["pitchbend", "aftertouch"].contains(e.type) {
                    row(e.type == "note" ? "Note" : (e.type == "pc" ? "Program" : "CC")) {
                        NumberField(value: numberBinding, range: 0...(e.type == "cc14" ? 31 : 127))
                    }
                }
                let maxV = EC4Names.highres(e) ? 4095 : 255
                row("Lower → upper") {
                    HStack(spacing: 4) {
                        NumberField(value: eb(\.lower), range: 0...maxV, width: 52)
                        Image(systemName: "arrow.right").foregroundStyle(Theme.dim).imageScale(.small)
                        NumberField(value: eb(\.upper), range: 0...maxV, width: 52)
                    }
                }
                row("Display") {
                    Picker("", selection: eb(\.scale)) { ForEach(EC4Names.scales, id: \.0) { Text($0.1).tag($0.0) } }
                        .labelsHidden().frame(width: 170)
                }
                row("Acceleration") {
                    Picker("", selection: eb(\.mode)) { ForEach(EC4Names.modes, id: \.0) { Text($0.1).tag($0.0) } }
                        .labelsHidden().frame(width: 170)
                }
                Toggle("Link to the next encoder", isOn: eb(\.link)).toggleStyle(.switch)
            }
            if e.type == "cc" {
                Card(title: "Send to ER-301", accent: Theme.cv) {
                    let port = mapping.faderPort(channel: e.channel, cc: e.number)
                    Toggle("Route through the i2c2midi", isOn: Binding(
                        get: { port != nil },
                        set: { on in mapping.setFaderPort(channel: e.channel, cc: e.number, port: on ? mapping.firstFreePort() : nil) }))
                        .toggleStyle(.switch)
                    if let port {
                        row("SC.CV port") {
                            NumberField(value: Binding(get: { port }, set: { mapping.setFaderPort(channel: e.channel, cc: e.number, port: $0) }), range: 1...99)
                        }
                    }
                    Hint(text: "Needs the EC4 on SSH (the MIDI hub on the SSH host passes its MIDI to the i2c2midi). The route is stored with the ER-301 routing, so it applies in every setup that sends this channel + CC.")
                }
            }
            HStack(spacing: 6) {
                Button("Channel \(e.channel) on all 16") {
                    for k in 0..<16 { setCh(k, e.channel) }
                }
                Button("Numbers ascending from here") {
                    for k in 0..<16 { setNum(k, min(127, max(0, e.number - selected + k))) }
                }.disabled(e.number - selected < 0 || e.number - selected + 15 > 127)
            }.controlSize(.small)
        }
    }

    @ViewBuilder private var pushEditor: some View {
        let p = enc.push
        VStack(alignment: .leading, spacing: 12) {
            Text("Push \(selected + 1)").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            NoteField(text: notes.binding(NotesStore.ec4Push(setup, group, selected)), title: "Note · push button")
            Card(title: "Push button", accent: Theme.trig) {
                row("Type") {
                    Picker("", selection: eb(\.push.type)) { ForEach(EC4Names.pushTypes, id: \.0) { Text($0.1).tag($0.0) } }
                        .labelsHidden().frame(width: 170)
                }
                if ["note", "cc", "pc", "pitchbend", "aftertouch"].contains(p.type) {
                    row("Channel") { NumberField(value: eb(\.push.channel), range: 1...16) }
                    if ["note", "cc", "pc"].contains(p.type) {
                        row(p.type == "note" ? "Note" : (p.type == "pc" ? "Program" : "CC")) { NumberField(value: eb(\.push.number), range: 0...127) }
                    }
                    row("Mode") {
                        Picker("", selection: eb(\.push.mode)) { Text("Key (momentary)").tag("key"); Text("Toggle").tag("toggle") }
                            .labelsHidden().frame(width: 170)
                    }
                    row("Released → pressed") {
                        HStack(spacing: 4) {
                            NumberField(value: eb(\.push.lower), range: 0...127)
                            Image(systemName: "arrow.right").foregroundStyle(Theme.dim).imageScale(.small)
                            NumberField(value: eb(\.push.upper), range: 0...127)
                        }
                    }
                    Toggle("Show on the display", isOn: eb(\.push.display)).toggleStyle(.switch)
                    Toggle("Link to the next push button", isOn: eb(\.push.link)).toggleStyle(.switch)
                } else if p.type == "group" || p.type == "setup" {
                    row(p.type == "group" ? "Group" : "Setup") {
                        NumberField(value: Binding(get: { p.number + 1 }, set: { board.setups[setup].groups[group].encoders[selected].push.number = max(0, min(15, $0 - 1)) }), range: 1...16)
                    }
                }
            }
        }
    }

    @ViewBuilder private var namesEditor: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Names").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            Card(title: "Setup \(setup + 1)") {
                row("Setup name") { NameField(text: Binding(get: { board.setups[setup].name }, set: { board.setups[setup].name = $0 })) }
            }
            Card(title: "Groups of this setup") {
                ForEach(0..<16, id: \.self) { g in
                    row("Group \(g + 1)") {
                        NameField(text: Binding(get: { board.setups[setup].groups[g].name }, set: { board.setups[setup].groups[g].name = $0 }))
                    }
                }
            }
            Hint(text: "Names are 4 characters, as on the EC4's display.")
        }
    }

    // channel / number changes move an existing ER-301 route along
    private var channelBinding: Binding<Int> { Binding(get: { enc.channel }, set: { setCh(selected, $0) }) }
    private var numberBinding: Binding<Int> { Binding(get: { enc.number }, set: { setNum(selected, $0) }) }

    private func moveRoute(_ k: Int, ch: Int, num: Int) {
        let old = board.setups[setup].groups[group].encoders[k]
        guard old.type == "cc", (old.channel, old.number) != (ch, num), let p = mapping.faderPort(channel: old.channel, cc: old.number) else { return }
        mapping.setFaderPort(channel: old.channel, cc: old.number, port: nil)
        mapping.setFaderPort(channel: ch, cc: num, port: p)
    }
    private func setCh(_ k: Int, _ ch: Int) {
        moveRoute(k, ch: ch, num: board.setups[setup].groups[group].encoders[k].number)
        board.setups[setup].groups[group].encoders[k].channel = ch
    }
    private func setNum(_ k: Int, _ n: Int) {
        moveRoute(k, ch: board.setups[setup].groups[group].encoders[k].channel, num: n)
        board.setups[setup].groups[group].encoders[k].number = n
    }

    private func row<V: View>(_ title: String, @ViewBuilder _ v: () -> V) -> some View {
        HStack {
            Text(title).font(Theme.font(12, .medium, .ui)).foregroundStyle(Theme.ink)
            Spacer()
            v()
        }
    }
}

/// 4-character name field (EC4 display).
struct NameField: View {
    @Binding var text: String
    var body: some View {
        TextField("", text: Binding(get: { text.trimmingCharacters(in: .whitespaces) },
                                    set: { text = String($0.prefix(4)) }))
            .font(Theme.font(13, .bold, .mono))
            .multilineTextAlignment(.center)
            .frame(width: 70)
            .textFieldStyle(.roundedBorder)
    }
}
