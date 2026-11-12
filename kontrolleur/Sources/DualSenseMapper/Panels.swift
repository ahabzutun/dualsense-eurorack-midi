import SwiftUI

// MARK: - Small building blocks

@MainActor
struct NumberField: View {
    @Binding var value: Int
    let range: ClosedRange<Int>
    var width: CGFloat = 46

    var body: some View {
        HStack(spacing: 4) {
            TextField("", value: Binding(get: { value }, set: { value = min(max($0, range.lowerBound), range.upperBound) }),
                      format: .number.grouping(.never))
                .textFieldStyle(.plain)
                .font(Theme.font(13, .semibold, .mono))
                .multilineTextAlignment(.trailing)
                .frame(width: width)
                .padding(.vertical, 4).padding(.horizontal, 6)
                .background(RoundedRectangle(cornerRadius: 6).fill(Theme.field))
            Stepper("", value: $value, in: range).labelsHidden()
        }
    }
}

@MainActor
struct Card<Content: View>: View {
    let title: String
    var accent: Color = Theme.ink
    @ViewBuilder var content: Content

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text(title.uppercased())
                .font(Theme.font(10, .heavy, .mono)).tracking(1.2)
                .foregroundStyle(accent.opacity(0.9))
            content
        }
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(RoundedRectangle(cornerRadius: 12).fill(Theme.card))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(Theme.line))
    }
}

struct Hint: View {
    let text: String
    var body: some View {
        Text(text).font(Theme.font(11, .regular, .ui)).foregroundStyle(Theme.dim).fixedSize(horizontal: false, vertical: true)
    }
}

// MARK: - Side panel

enum PanelTab: String, CaseIterable, Identifiable {
    case control = "Control", combos = "Combos", overview = "Overview"
    var id: String { rawValue }
}

@MainActor
struct SidePanel: View {
    @Binding var selected: GroupID
    @Binding var tab: PanelTab

    var body: some View {
        VStack(spacing: 0) {
            Picker("", selection: $tab) {
                ForEach(PanelTab.allCases) { Text($0.rawValue).tag($0) }
            }
            .pickerStyle(.segmented).labelsHidden()
            .padding(14)
            Divider().overlay(Theme.line)
            ScrollView {
                Group {
                    switch tab {
                    case .control: ControlPanel(group: Catalog.group(selected))
                    case .combos: CombosPanel()
                    case .overview: OverviewPanel()
                    }
                }
                .padding(14)
            }
        }
        .frame(width: 384)
        .background(Theme.panel)
    }
}

// MARK: Control

@MainActor
struct ControlPanel: View {
    @Environment(Mapping.self) private var mapping
    @Environment(NotesStore.self) private var notes
    let group: GroupDef

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            VStack(alignment: .leading, spacing: 6) {
                Text(group.title).font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
                Hint(text: group.hint)
            }
            .padding(.bottom, 4)
            ForEach(group.inputs) { input in
                InputEditor(input: input)
                NoteField(text: notes.binding(NotesStore.ds(input.id)),
                          title: group.inputs.count > 1 ? "Note · \(input.label)" : "Note")
            }
        }
    }
}

@MainActor
struct InputEditor: View {
    @Environment(Mapping.self) private var mapping
    let input: InputDef

    var body: some View {
        let role = mapping.role(input)
        let sends = mapping.sendsMidi(input)
        let hasFunction = input.button != nil || input.id == "dpad_horizontal" || input.id == "left_stick_x"
        Card(title: input.label, accent: accentColor) {
            if let b = input.button { functionSection(b) }
            if input.id == "dpad_horizontal" {
                modePicker([("slice_section", "Slice section ±16"), ("midi", "MIDI CC steps")],
                           Binding(get: { mapping.dpadHorizontal }, set: { mapping.dpadHorizontal = $0 }))
            }
            if input.id == "left_stick_x" {
                modePicker([("slice_fine", "Slice in section"), ("midi", "MIDI CC")],
                           Binding(get: { mapping.leftStickX }, set: { mapping.leftStickX = $0 }))
            }
            if sends {
                if hasFunction {
                    Divider().overlay(Theme.line)
                    Text("SEND TO ER-301").font(Theme.font(9, .heavy, .mono)).tracking(1)
                        .foregroundStyle(Theme.dim)
                }
                Picker("", selection: Binding(get: { mapping.role(input) }, set: { mapping.setRole($0, for: input) })) {
                    ForEach(input.roles) { Text($0.title).tag($0) }
                }
                .pickerStyle(.segmented).labelsHidden()

                if role.usesPort {
                    HStack {
                        Text(role == .cv ? "SC.CV port" : "\(mapping.triggerPortName) port")
                            .font(Theme.font(12, .medium, .ui)).foregroundStyle(Theme.ink)
                        Spacer()
                        NumberField(value: Binding(get: { mapping.port(input) }, set: { mapping.ports[input.id] = $0 }), range: 1...99)
                    }
                }
                Hint(text: explanation(role))
            }
            Text(source)
                .font(Theme.font(10, .regular, .mono)).foregroundStyle(Theme.dim.opacity(0.8))
        }
    }

    private var accentColor: Color {
        for (_, k) in mapping.tokens(input) {
            if case .off = k { continue }
            return Theme.color(k)
        }
        return Theme.dim
    }

    @ViewBuilder private func functionSection(_ b: String) -> some View {
        let f = mapping.fn(b)
        HStack {
            Text("Tap").font(Theme.font(12, .semibold, .ui)).foregroundStyle(Theme.ink)
            Spacer()
            Picker("", selection: Binding(get: { mapping.fn(b).tap }, set: { mapping.setTap($0, b) })) {
                ForEach(Action.allCases) { Text($0.title).tag($0) }
            }
            .labelsHidden().frame(width: 230)
        }
        HStack {
            Text("Hold").font(Theme.font(12, .semibold, .ui)).foregroundStyle(Theme.ink)
            Spacer()
            Picker("", selection: Binding(get: { mapping.fn(b).hold }, set: { mapping.setHold($0, b) })) {
                ForEach(Action.holdChoices) { Text($0.title).tag($0) }
            }
            .labelsHidden().frame(width: 230)
        }
        Hint(text: timing(f))
        let combos = mapping.chords.filter { $0.a == b || $0.b == b }
        if !combos.isEmpty {
            Text(combos.map { "Combo \(Buttons.label($0.a)) + \(Buttons.label($0.b)) → \($0.action.title)" }.joined(separator: "\n"))
                .font(Theme.font(11, .regular, .ui)).foregroundStyle(Theme.function.opacity(0.85))
        }
    }

    private func timing(_ f: ButtonFunc) -> String {
        let hold = String(format: "%g", mapping.longPress)
        if f.hold != .none {
            return "Shorter than \(hold) s = tap, longer = hold; decided when you let go."
        }
        if f.tap == .midi { return "Sends its CC: 127 while pressed, 0 on release." }
        if f.tap == .none { return "Does nothing." }
        return f.tap.firesOnRelease ? "Fires when you let go." : "Fires when you press."
    }

    private func modePicker(_ options: [(String, String)], _ value: Binding<String>) -> some View {
        Picker("", selection: value) {
            ForEach(options, id: \.0) { Text($0.1).tag($0.0) }
        }
        .pickerStyle(.segmented).labelsHidden()
    }

    private var source: String {
        let n = mapping.number(input)
        switch input.kind {
        case .cc: return "MIDI ch \(mapping.channel) · CC \(n)" + (input.name.map { " · \($0)" } ?? "")
        case .nrpn: return "MIDI ch \(mapping.channel) · NRPN \(n) (14-bit)" + (input.name.map { " · \($0)" } ?? "")
        case .note: return "MIDI ch \(mapping.channel) · note \(n)"
        }
    }

    private func explanation(_ r: Role) -> String {
        switch r {
        case .off: return "Not sent to the ER-301."
        case .cv: return input.kind == .nrpn ? "0–16383 → 0–10 V in full resolution." : "0–127 → 0–10 V."
        case .trigger: return mapping.triggersAsCV ? "10 V while pressed, 0 V when released (on an SC.CV port)." : "SC.TR high while pressed."
        case .noteTrigger: return mapping.triggersAsCV ? "10 V while the note is held (on an SC.CV port)." : "SC.TR high while the note is held."
        case .lfoRate: return "Sets the trigger LFO speed. 0 = stopped. Port and range: Pitch/LFO tab. Only one control can do this."
        case .lfoReset: return "Pressing restarts the LFO phase (a trigger right away). Only one control can do this."
        case .octave: return "Shifts the pitch CV by \(mapping.semisPerStep) semitones per step. Only one control can do this."
        }
    }
}

// MARK: Pitch & LFO

@MainActor
struct PitchLfoPanel: View {
    @Environment(Mapping.self) private var mapping

    private static let noteNames = ["C", "C♯", "D", "D♯", "E", "F", "F♯", "G", "G♯", "A", "A♯", "B"]
    private func noteName(_ n: Int) -> String { "\(Self.noteNames[n % 12])\(n / 12 - 1)" }

    var body: some View {
        @Bindable var m = mapping
        VStack(alignment: .leading, spacing: 12) {
            Card(title: "Pitch · V/Oct", accent: Theme.cv) {
                Toggle("Notes → pitch CV", isOn: $m.pitchEnabled).toggleStyle(.switch)
                if m.pitchEnabled {
                    row("SC.CV port") { NumberField(value: $m.pitchPort, range: 1...99) }
                    row("0 V note") {
                        HStack(spacing: 8) {
                            Text(noteName(m.zeroNote)).font(Theme.font(12, .semibold, .mono)).foregroundStyle(Theme.cv)
                            NumberField(value: $m.zeroNote, range: 0...127)
                        }
                    }
                    row("Semitones per octave step") { NumberField(value: $m.semisPerStep, range: 0...48) }
                    row("Octave control") {
                        Text(mapping.input(with: .octave).map { Catalog.groupOf($0).title + " " + $0.label.lowercased() } ?? "none")
                            .font(Theme.font(12, .regular, .mono)).foregroundStyle(Theme.octave)
                    }
                    Hint(text: "Pick the octave control on the D-pad (Control tab → Octave).")
                }
            }

            Card(title: "Trigger LFO", accent: Theme.special) {
                row("Rate control") {
                    Picker("", selection: lfoBinding(.lfoRate)) {
                        Text("none (LFO off)").tag("")
                        ForEach(Catalog.allInputs.filter { $0.roles.contains(.lfoRate) }) { Text(label($0)).tag($0.id) }
                    }.labelsHidden().frame(width: 190)
                }
                row("Phase reset") {
                    Picker("", selection: lfoBinding(.lfoReset)) {
                        Text("none").tag("")
                        ForEach(Catalog.allInputs.filter { $0.roles.contains(.lfoReset) }) { Text(label($0)).tag($0.id) }
                    }.labelsHidden().frame(width: 190)
                }
                row("\(mapping.triggerPortName) port") { NumberField(value: $m.lfoPort, range: 1...99) }
                row("Slowest (Hz)") { hzField($m.lfoMinHz) }
                row("Fastest (Hz)") { hzField($m.lfoMaxHz) }
                row("Pulse length (ms)") { NumberField(value: $m.lfoPulseMs, range: 1...127) }
                Hint(text: "Exponential curve: finer control at slow speeds. Pulses get shorter automatically at fast rates.")
            }

            Card(title: "Output") {
                Toggle("Triggers as SC.CV (0 / 10 V)", isOn: $m.triggersAsCV).toggleStyle(.switch)
                Hint(text: m.triggersAsCV
                     ? "Trigger ports are SC.CV ports. Use this when the ER-301 trigger input's source is an SC.CV."
                     : "Trigger ports are SC.TR ports (TR / TR.PULSE).")
            }
        }
    }

    private func label(_ i: InputDef) -> String {
        let g = Catalog.groupOf(i)
        return g.inputs.count > 1 ? "\(g.title) \(i.label.lowercased())" : g.title
    }

    private func lfoBinding(_ r: Role) -> Binding<String> {
        Binding(get: { mapping.input(with: r)?.id ?? "" },
                set: { id in
                    if let cur = mapping.input(with: r) { mapping.roles[cur.id] = .off }
                    if let i = Catalog.allInputs.first(where: { $0.id == id }) { mapping.setRole(r, for: i) }
                })
    }

    private func hzField(_ v: Binding<Double>) -> some View {
        TextField("", value: v, format: .number.precision(.fractionLength(0...2)))
            .textFieldStyle(.plain)
            .font(Theme.font(13, .semibold, .mono))
            .multilineTextAlignment(.trailing)
            .frame(width: 70)
            .padding(.vertical, 4).padding(.horizontal, 6)
            .background(RoundedRectangle(cornerRadius: 6).fill(Theme.field))
    }

    private func row<V: View>(_ title: String, @ViewBuilder _ v: () -> V) -> some View {
        HStack {
            Text(title).font(Theme.font(12, .medium, .ui)).foregroundStyle(Theme.ink)
            Spacer()
            v()
        }
    }
}

// MARK: Other MIDI sources

@MainActor
struct ExternalPanel: View {
    @Environment(Mapping.self) private var mapping

    var body: some View {
        @Bindable var m = mapping
        VStack(alignment: .leading, spacing: 12) {
            Text("MIDI routes").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            Hint(text: "Everything from the MIDI hub that goes to the ER-301 besides the DualSense: 16n, pedals, EC4 or any other source. The device tabs add and remove their routes here for you; you can also add routes by hand (NRPN, notes, triggers, other channels). Control is a CC / NRPN / note number, or a name from mappings.py.")
            ForEach($m.externals) { $e in
                Card(title: e.kind.rawValue, accent: e.kind == .cv || e.kind == .nrpn ? Theme.cv : Theme.trig) {
                    HStack {
                        Picker("", selection: $e.kind) { ForEach(ExternalRoute.Kind.allCases) { Text($0.rawValue).tag($0) } }
                            .labelsHidden().frame(width: 150)
                        Spacer()
                        Button(role: .destructive) { m.externals.removeAll { $0.id == e.id } } label: { Image(systemName: "trash") }
                            .buttonStyle(.borderless)
                    }
                    HStack {
                        Text("Channel").font(Theme.font(12, .regular, .ui)).foregroundStyle(Theme.ink)
                        Spacer()
                        NumberField(value: $e.channel, range: 1...16)
                    }
                    HStack {
                        Text(e.kind == .note ? "Note" : e.kind == .nrpn ? "NRPN" : "CC").font(Theme.font(12, .regular, .ui)).foregroundStyle(Theme.ink)
                        Spacer()
                        TextField("", text: $e.control)
                            .textFieldStyle(.plain).font(Theme.font(13, .semibold, .mono))
                            .multilineTextAlignment(.trailing).frame(width: 140)
                            .padding(.vertical, 4).padding(.horizontal, 6)
                            .background(RoundedRectangle(cornerRadius: 6).fill(Theme.field))
                    }
                    HStack {
                        Text(e.kind == .cv || e.kind == .nrpn ? "SC.CV port" : "\(mapping.triggerPortName) port")
                            .font(Theme.font(12, .regular, .ui)).foregroundStyle(Theme.ink)
                        Spacer()
                        NumberField(value: $e.port, range: 1...99)
                    }
                }
            }
            Button {
                m.externals.append(ExternalRoute(kind: .cv, channel: m.channel, control: "109", port: m.firstFreePort()))
            } label: {
                Label("Add route", systemImage: "plus.circle.fill").frame(maxWidth: .infinity)
            }
            .controlSize(.large)
        }
    }
}

// MARK: Combos & overview

@MainActor
struct CombosPanel: View {
    @Environment(Mapping.self) private var mapping

    var body: some View {
        @Bindable var m = mapping
        VStack(alignment: .leading, spacing: 12) {
            Text("Combos").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            Hint(text: "Two buttons pressed together. Their own release actions are skipped then, so L1 + R1 doesn't also delete or play the loop.")
            if !m.controlsOnPi {
                Hint(text: "⚠︎ The SSH host doesn't have the button functions installed yet: only the ER-301 routing is uploaded.")
            }
            ForEach($m.chords) { $c in
                Card(title: "\(Buttons.label(c.a)) + \(Buttons.label(c.b))", accent: Theme.function) {
                    HStack(spacing: 8) {
                        buttonPicker($c.a)
                        Text("+").foregroundStyle(Theme.dim)
                        buttonPicker($c.b)
                        Spacer()
                        Button(role: .destructive) { m.chords.removeAll { $0.id == c.id } } label: { Image(systemName: "trash") }
                            .buttonStyle(.borderless)
                    }
                    Picker("", selection: $c.action) {
                        ForEach(Action.chordChoices) { Text($0.title).tag($0) }
                    }
                    .labelsHidden()
                }
            }
            Button {
                m.chords.append(Chord(a: "triangle", b: "square", action: .motion))
            } label: {
                Label("Add combo", systemImage: "plus.circle.fill").frame(maxWidth: .infinity)
            }
            .controlSize(.large)
        }
    }

    private func buttonPicker(_ value: Binding<String>) -> some View {
        Picker("", selection: value) {
            ForEach(Buttons.all, id: \.name) { Text($0.label).tag($0.name) }
        }
        .labelsHidden().frame(width: 110)
    }
}

// MARK: DualSense overview

@MainActor
struct OverviewPanel: View {
    @Environment(Mapping.self) private var mapping

    var body: some View {
        @Bindable var m = mapping
        VStack(alignment: .leading, spacing: 12) {
            Text("DualSense").font(Theme.font(24, .bold, .title)).foregroundStyle(Theme.ink)
            Card(title: "MIDI") {
                HStack {
                    Text("MIDI channel").font(Theme.font(12, .medium, .ui)).foregroundStyle(Theme.ink)
                    Spacer()
                    NumberField(value: $m.channel, range: 1...16)
                }
                Hint(text: "The instrument channel the DualSense sends on (pedals on channel 1 follow it in the MIDI hub). The ER-301 routes use it as CHANNEL.")
            }
            Card(title: "All buttons", accent: Theme.function) {
                ForEach(Buttons.all, id: \.name) { b in
                    let f = m.fn(b.name)
                    HStack(alignment: .firstTextBaseline) {
                        Text(b.label).font(Theme.font(12, .bold, .mono)).foregroundStyle(Theme.ink)
                            .frame(width: 92, alignment: .leading)
                        VStack(alignment: .leading, spacing: 1) {
                            Text(f.tap == .midi ? "MIDI → " + (Catalog.input(forButton: b.name).map { m.routeToken($0) } ?? "")
                                                : f.tap.title)
                                .font(Theme.font(12, .regular, .ui)).foregroundStyle(f.tap == .none ? Theme.dim : Theme.ink)
                            if f.hold != .none {
                                Text("hold: " + f.hold.title).font(Theme.font(11, .regular, .ui)).foregroundStyle(Theme.function)
                            }
                        }
                    }
                }
                Hint(text: "Hold = \(String(format: "%g", m.longPress)) s or longer (LONG_PRESS_DURATION in mappings.py).")
            }
        }
    }

}
