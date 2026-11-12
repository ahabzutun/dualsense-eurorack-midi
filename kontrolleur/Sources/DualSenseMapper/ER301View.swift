import SwiftUI

// MARK: - Which device feeds a port

struct PortSource: Hashable {
    let device: String      // DualSense, 16n, MPC-20, EC4, i2c2midi, MIDI
    let detail: String      // fader 3, EXP3 · message 1, STP1 · VECT · VE23 …
    var note: String? = nil
}

@MainActor
struct SourceResolver {
    let mapping: Mapping
    let bank: Faderbank
    let pedals: PedalBoard
    let ec4: EC4Board
    let notes: NotesStore

    func describe(_ r: Mapping.PortRef) -> [PortSource] {
        switch r {
        case .dualsense(let label, let id, let trig):
            return [PortSource(device: "DualSense", detail: label + (trig ? " (trigger)" : ""), note: notes.note(NotesStore.ds(id)))]
        case .builtin(let name):
            let detail = name == "LFO" ? "trigger LFO" : "pitch from DualSense notes"
            return [PortSource(device: "i2c2midi", detail: detail)]
        case .external(let kind, let ch, let control):
            var out: [PortSource] = []
            if kind == "cc" || kind == "trigger", let cc = Int(control) {
                for (i, f) in bank.faders.enumerated() where f.usb_channel == ch && f.usb_cc == cc {
                    out.append(PortSource(device: "16n", detail: "fader \(i + 1)", note: notes.note(NotesStore.fader(i))))
                }
                for inp in pedals.inputs {
                    for s in inp.slots where s.on && s.kind == "cc" && s.number == cc && PedalBoard.hubChannel(s.channel, mapping: mapping) == ch {
                        out.append(PortSource(device: "MPC-20", detail: "\(PedalBoard.name(inp.input)) · message \(s.slot + 1)",
                                              note: notes.note(NotesStore.pedal(inp.input, s.slot))))
                    }
                }
                var ec4Hits: [PortSource] = []
                for (si, su) in ec4.setups.enumerated() {
                    for (gi, g) in su.groups.enumerated() {
                        for (ei, e) in g.encoders.enumerated() where e.type == "cc" && e.channel == ch && e.number == cc {
                            let n = e.name.trimmingCharacters(in: .whitespaces)
                            ec4Hits.append(PortSource(device: "EC4",
                                                      detail: "\(su.name.trimmingCharacters(in: .whitespaces)) · \(g.name.trimmingCharacters(in: .whitespaces)) · \(n.isEmpty ? "enc \(ei + 1)" : n)",
                                                      note: notes.note(NotesStore.ec4(si, gi, ei))))
                        }
                    }
                }
                if let first = ec4Hits.first {
                    out.append(ec4Hits.count == 1 ? first
                               : PortSource(device: "EC4", detail: first.detail + " +\(ec4Hits.count - 1) more", note: first.note))
                }
            }
            if out.isEmpty {
                let what = kind == "note" ? "note" : kind == "nrpn" ? "NRPN" : "CC"
                out.append(PortSource(device: "MIDI", detail: "ch \(ch) \(what) \(control)" + (kind == "trigger" ? " (trigger)" : "")))
            }
            return out
        }
    }

    func sources(_ use: Mapping.PortUse) -> [PortSource] { use.refs.flatMap(describe) }

    static func color(_ device: String) -> Color {
        switch device {
        case "DualSense": Theme.cv
        case "16n": Theme.special
        case "MPC-20": Theme.trig
        case "EC4": Theme.octave
        case "i2c2midi": Theme.function
        default: Theme.dim
        }
    }
}

extension SourceResolver {
    /// Distinct colours of a port's sources, in order.
    static func colors(_ sources: [PortSource]) -> [Color] {
        var seen: [String] = []
        for s in sources where !seen.contains(s.device) { seen.append(s.device) }
        return seen.map(color)
    }

    /// Hard-edged bands, one per colour (a port fed by DualSense + 16n is half cyan, half green).
    static func bands(_ cols: [Color], opacity: Double = 1) -> LinearGradient {
        let c = cols.isEmpty ? [Theme.dim] : cols
        var stops: [Gradient.Stop] = []
        for (i, col) in c.enumerated() {
            let a = Double(i) / Double(c.count), b = Double(i + 1) / Double(c.count)
            stops.append(.init(color: col.opacity(opacity), location: a))
            stops.append(.init(color: col.opacity(opacity), location: b))
        }
        return LinearGradient(stops: stops, startPoint: .topLeading, endPoint: .bottomTrailing)
    }
}

/// "shared ×2": several sources drive the same port.
struct SharedBadge: View {
    let count: Int
    var body: some View {
        Text("×\(count)")
            .font(Theme.font(9, .heavy, .mono))
            .foregroundStyle(Theme.ink)
            .padding(.horizontal, 4).padding(.vertical, 1)
            .background(Capsule().stroke(Theme.ink.opacity(0.45), lineWidth: 1))
            .help("\(count) sources share this port. On a CV port the value sent last wins; on a trigger port each source fires it.")
            .fixedSize()
    }
}

struct DeviceBadge: View {
    let device: String
    var body: some View {
        Text(device)
            .font(Theme.font(9, .heavy, .mono))
            .foregroundStyle(SourceResolver.color(device))
            .padding(.horizontal, 5).padding(.vertical, 1.5)
            .background(Capsule().fill(SourceResolver.color(device).opacity(0.15)))
            .fixedSize()
    }
}

// MARK: - ER-301 tab drawing: all 100 SC.CV ports

@MainActor
struct ER301Canvas: View {
    @Environment(Mapping.self) private var mapping
    @Environment(Faderbank.self) private var bank
    @Environment(PedalBoard.self) private var pedals
    @Environment(EC4Board.self) private var ec4
    @Environment(NotesStore.self) private var notes
    @Binding var selectedPort: String?

    var body: some View {
        // A flow layout: tiles size themselves to their text, the stage grows with it
        ZoomStage(size: CGSize(width: 1100, height: 640), grows: true) {
            let map = mapping.portMap
            let used = Dictionary(uniqueKeysWithValues: map.map { ($0.id, $0) })
            let resolver = SourceResolver(mapping: mapping, bank: bank, pedals: pedals, ec4: ec4, notes: notes)
            VStack(alignment: .leading, spacing: 10) {
                HStack(alignment: .firstTextBaseline, spacing: 12) {
                    Text("ER-301").font(Theme.font(22, .heavy, .title)).foregroundStyle(Theme.ink)
                    Text("SC.CV 1–99 · \(map.filter { $0.kind == "CV" }.count) in use")
                        .font(Theme.font(11, .semibold, .mono)).foregroundStyle(Theme.dim)
                    Spacer(minLength: 20)
                    ForEach(["DualSense", "16n", "MPC-20", "EC4", "i2c2midi", "MIDI"], id: \.self) { DeviceBadge(device: $0) }
                }
                .frame(maxWidth: .infinity)
                portGrid(kind: "CV", ports: Array(1...99), used: used, resolver: resolver)
                if !mapping.triggersAsCV {
                    Text("SC.TR").font(Theme.font(11, .heavy, .mono)).foregroundStyle(Theme.trig)
                    portGrid(kind: "TR", ports: Array(1...10), used: used, resolver: resolver)
                }
            }
        }
    }

    /// Ten ports per row; every tile in a row gets the height of the tallest one.
    private func portGrid(kind: String, ports: [Int], used: [String: Mapping.PortUse], resolver: SourceResolver) -> some View {
        Grid(alignment: .topLeading, horizontalSpacing: 5, verticalSpacing: 5) {
            ForEach(Array(stride(from: 0, to: ports.count, by: 10)), id: \.self) { start in
                GridRow {
                    ForEach(ports[start..<min(start + 10, ports.count)], id: \.self) { p in
                        let id = "\(kind)\(p)"
                        PortTile(kind: kind, port: p, sources: used[id].map(resolver.sources) ?? [], selected: selectedPort == id)
                            .onTapGesture { selectedPort = (selectedPort == id ? nil : id) }
                    }
                }
            }
        }
    }
}

@MainActor
struct PortTile: View {
    let kind: String
    let port: Int
    let sources: [PortSource]
    let selected: Bool

    var body: some View {
        let cols = SourceResolver.colors(sources)
        let col = cols.first ?? Theme.dim
        VStack(alignment: .leading, spacing: 2) {
            HStack(spacing: 3) {
                Text("\(port)").font(Theme.font(11, .heavy, .mono))
                    .foregroundStyle(sources.isEmpty ? Theme.dim.opacity(0.7) : col)
                Spacer(minLength: 0)
                if sources.count > 1 { SharedBadge(count: sources.count) }
            }
            if let s = sources.first {
                Text(Array(Set(sources.map(\.device))).sorted().joined(separator: " + ")).font(Theme.font(8, .heavy, .mono)).foregroundStyle(col).lineLimit(1).minimumScaleFactor(0.8)
                Text(s.note ?? s.detail).font(Theme.font(8.5, .medium)).foregroundStyle(s.note == nil ? Theme.ink : Theme.note).lineLimit(1).minimumScaleFactor(0.8)
            }
            Spacer(minLength: 0)
        }
        .padding(5)
        .frame(width: 100 * Theme.grow, alignment: .topLeading)
        .frame(minHeight: 52 * Theme.grow, maxHeight: .infinity, alignment: .topLeading)
        .background(RoundedRectangle(cornerRadius: 7).fill(sources.isEmpty ? AnyShapeStyle(Color.clear)
                                                                          : AnyShapeStyle(SourceResolver.bands(cols, opacity: 0.16))))
        .overlay {
            if sources.isEmpty {
                RoundedRectangle(cornerRadius: 7).stroke(selected ? Theme.ink.opacity(0.8) : Theme.body.opacity(0.15),
                                                         style: StrokeStyle(lineWidth: selected ? 1.6 : 1, dash: [2, 4]))
            } else if selected {
                RoundedRectangle(cornerRadius: 7).stroke(Theme.ink.opacity(0.8), lineWidth: 1.6)
            } else {
                RoundedRectangle(cornerRadius: 7).stroke(SourceResolver.bands(cols, opacity: 0.6), lineWidth: 1.2)
            }
        }
        .contentShape(Rectangle())
        .help(sources.isEmpty ? "SC.\(kind) \(port): free"
              : "SC.\(kind) \(port)\n" + sources.map { "\($0.device): \($0.detail)" + ($0.note.map { " — \($0)" } ?? "") }.joined(separator: "\n"))
    }
}

// MARK: - Port list (inspector)

@MainActor
struct PortMapPanel: View {
    @Environment(Mapping.self) private var mapping
    @Environment(Faderbank.self) private var bank
    @Environment(PedalBoard.self) private var pedals
    @Environment(EC4Board.self) private var ec4
    @Environment(NotesStore.self) private var notes
    var selectedPort: String? = nil

    var body: some View {
        let resolver = SourceResolver(mapping: mapping, bank: bank, pedals: pedals, ec4: ec4, notes: notes)
        VStack(alignment: .leading, spacing: 12) {
            Text("ER-301 ports").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            Hint(text: "Set these as the sources of your ER-301 unit inputs. Each entry shows which device the MIDI comes from. “×2” marks a port with several sources: fine when intended (e.g. ✕ and the LFO both firing the grain trigger); on a CV port the value sent last wins.")
            let map = mapping.portMap
            if map.isEmpty { Hint(text: "Nothing routed yet.") }
            ForEach(map) { use in
                let srcs = resolver.sources(use)
                let isSel = selectedPort == use.id
                HStack(alignment: .top, spacing: 10) {
                    Text("SC.\(use.kind) \(use.port)")
                        .font(Theme.font(13, .heavy, .mono))
                        .foregroundStyle(use.kind == "CV" ? Theme.cv : Theme.trig)
                        .frame(width: 84, alignment: .leading)
                    VStack(alignment: .leading, spacing: 5) {
                        ForEach(srcs, id: \.self) { s in
                            VStack(alignment: .leading, spacing: 1) {
                                HStack(spacing: 6) {
                                    DeviceBadge(device: s.device)
                                    Text(s.detail).font(Theme.font(12)).foregroundStyle(Theme.ink).lineLimit(2)
                                }
                                if let n = s.note { NoteBadge(text: n, size: 10.5, lines: 2) }
                            }
                        }
                    }
                    Spacer(minLength: 0)
                    if srcs.count > 1 { SharedBadge(count: srcs.count) }
                }
                .padding(.vertical, 8).padding(.horizontal, 12)
                .background(RoundedRectangle(cornerRadius: 10).fill(Theme.card))
                .overlay(RoundedRectangle(cornerRadius: 10).stroke(isSel ? Theme.ink.opacity(0.7) : Color.clear, lineWidth: 1.2))
            }
        }
    }
}

// MARK: - Inspector

enum ER301Tab: String, CaseIterable, Identifiable {
    case ports = "Ports", routes = "Routes", pitchLfo = "Pitch & LFO"
    var id: String { rawValue }
}

@MainActor
struct ER301Panel: View {
    @Binding var tab: ER301Tab
    var selectedPort: String? = nil

    var body: some View {
        VStack(spacing: 0) {
            Picker("", selection: $tab) { ForEach(ER301Tab.allCases) { Text($0.rawValue).tag($0) } }
                .pickerStyle(.segmented).labelsHidden().padding(14)
            Divider().overlay(Theme.line)
            ScrollView {
                Group {
                    switch tab {
                    case .ports: PortMapPanel(selectedPort: selectedPort)
                    case .routes: ExternalPanel()
                    case .pitchLfo: PitchLfoPanel()
                    }
                }
                .padding(14)
            }
        }
        .frame(width: 384)
        .background(Theme.panel)
    }
}
