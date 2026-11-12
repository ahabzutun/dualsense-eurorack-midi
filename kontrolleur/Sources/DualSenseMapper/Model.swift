import Foundation
import Observation

// MARK: - ER-301 routing roles (what the i2c2midi does with a control's MIDI)

enum Role: String, CaseIterable, Identifiable {
    case off, cv, trigger, noteTrigger, lfoRate, lfoReset, octave
    var id: String { rawValue }

    var title: String {
        switch self {
        case .off: "Off"
        case .cv: "CV"
        case .trigger, .noteTrigger: "Trigger"
        case .lfoRate: "LFO rate"
        case .lfoReset: "LFO reset"
        case .octave: "Octave"
        }
    }
    var usesPort: Bool { self == .cv || self == .trigger || self == .noteTrigger }
    var isExclusive: Bool { self == .lfoRate || self == .lfoReset || self == .octave }
}

// MARK: - Button functions (what the DualSense script on the Pi does) — config/controls.py

enum Action: String, CaseIterable, Identifiable {
    case none, midi, loop_record, loop_play, loop_clear, motion, block_up, block_down, block_reset
    case section_up, section_down, strum, freeze_left, freeze_right, quantize
    var id: String { rawValue }

    var title: String {
        switch self {
        case .none: "Nothing"
        case .midi: "MIDI → ER-301"
        case .loop_record: "Loop: record start / stop"
        case .loop_play: "Loop: play / stop"
        case .loop_clear: "Loop: delete"
        case .motion: "Motion control on / off"
        case .block_up: "Slice block +128"
        case .block_down: "Slice block −128"
        case .block_reset: "Slice block → 0"
        case .section_up: "Slice section +16"
        case .section_down: "Slice section −16"
        case .strum: "Harmonic strum"
        case .freeze_left: "Freeze L2 / R2 / left stick"
        case .freeze_right: "Freeze right stick"
        case .quantize: "Loop quantize on / off"
        }
    }
    var token: String {
        switch self {
        case .none: "—"
        case .midi: "MIDI"
        case .loop_record: "REC"
        case .loop_play: "PLAY"
        case .loop_clear: "DELETE"
        case .motion: "MOTION"
        case .block_up: "BLOCK +"
        case .block_down: "BLOCK −"
        case .block_reset: "BLOCK 0"
        case .section_up: "SECTION +"
        case .section_down: "SECTION −"
        case .strum: "STRUM"
        case .freeze_left: "FREEZE L"
        case .freeze_right: "FREEZE R"
        case .quantize: "QUANTIZE"
        }
    }
    var firesOnRelease: Bool { self == .loop_record || self == .loop_play || self == .loop_clear }
    static let holdChoices: [Action] = allCases.filter { $0 != .midi }
    static let chordChoices: [Action] = allCases.filter { $0 != .midi && $0 != .none }
}

struct ButtonFunc: Equatable {
    var tap: Action
    var hold: Action      // .none = no hold action
}

struct Chord: Identifiable, Equatable {
    var id = UUID()
    var a: String
    var b: String
    var action: Action
}

enum Buttons {
    /// controls.py names, in file order, with display labels
    static let all: [(name: String, label: String)] = [
        ("cross", "✕"), ("circle", "○"), ("triangle", "△"), ("square", "□"),
        ("l1", "L1"), ("r1", "R1"), ("l3", "L3"), ("r3", "R3"),
        ("create", "Create"), ("options", "Options"), ("ps", "PS"), ("mic", "Mic"), ("touchpad", "Touchpad click"),
    ]
    static func label(_ n: String) -> String { all.first { $0.name == n }?.label ?? n }
    static let defaults: [String: ButtonFunc] = [
        "cross": .init(tap: .midi, hold: .none), "circle": .init(tap: .midi, hold: .none),
        "triangle": .init(tap: .midi, hold: .none), "square": .init(tap: .midi, hold: .none),
        "l1": .init(tap: .loop_clear, hold: .none), "r1": .init(tap: .loop_play, hold: .loop_record),
        "l3": .init(tap: .freeze_left, hold: .none), "r3": .init(tap: .freeze_right, hold: .none),
        "create": .init(tap: .block_down, hold: .none), "options": .init(tap: .block_up, hold: .none),
        "ps": .init(tap: .strum, hold: .none), "mic": .init(tap: .none, hold: .none),
        "touchpad": .init(tap: .quantize, hold: .none),
    ]
    static let defaultChords: [Chord] = [Chord(a: "l1", b: "r1", action: .motion),
                                         Chord(a: "create", b: "options", action: .block_reset)]
}

enum SourceKind { case cc, nrpn, note }

// MARK: - The controller

struct InputDef: Identifiable, Hashable {
    let id: String          // CC_MAP / NRPN_MAP name
    let label: String
    let kind: SourceKind
    let name: String?       // key in CC_MAP / NRPN_MAP on the Pi
    let fallback: Int       // CC / NRPN number if the name is unknown on the Pi
    let roles: [Role]
    var button: String? = nil   // controls.py button name for digital buttons

    /// Short name for the callouts on the controller drawing.
    var short: String {
        switch id {
        case "left_stick_x", "right_stick_x", "touchpad_x": return "X"
        case "left_stick_y", "right_stick_y", "touchpad_y": return "Y"
        case "btn_touchpad", "btn_l3", "btn_r3": return "CLICK"
        case "dpad_vertical": return "↕"
        case "dpad_horizontal": return "↔"
        default: return label.uppercased()
        }
    }
}

enum GroupID: String, CaseIterable, Identifiable {
    case l2, l1, r2, r1, dpad, leftStick, rightStick, touchpad, create, options, ps, mic
    case triangle, circle, cross, square, motion, slice
    var id: String { rawValue }
}

struct GroupDef: Identifiable {
    let id: GroupID
    let title: String
    let hint: String
    let inputs: [InputDef]
}

enum Catalog {
    static let axis: [Role] = [.off, .cv, .lfoRate]
    static let button: [Role] = [.off, .trigger, .cv, .lfoReset]

    static func cc(_ name: String, _ n: Int, _ label: String, _ roles: [Role]) -> InputDef {
        InputDef(id: name, label: label, kind: .cc, name: name, fallback: n, roles: roles)
    }
    static func btn(_ name: String, _ n: Int, _ button: String, _ label: String = "Button") -> InputDef {
        InputDef(id: name, label: label, kind: .cc, name: name, fallback: n, roles: Catalog.button, button: button)
    }
    static func nrpn(_ name: String, _ n: Int, _ label: String) -> InputDef {
        InputDef(id: "nrpn_" + name, label: label, kind: .nrpn, name: name, fallback: n, roles: [.off, .cv])
    }

    static let buttonHint = "Tap and hold can do different things. Set the tap to “MIDI → ER-301” to route the button like a trigger."

    static let groups: [GroupDef] = [
        GroupDef(id: .l2, title: "L2", hint: "Analog trigger: 0–127 → 0–10 V.",
                 inputs: [cc("l2_trigger", 7, "Pressure", axis)]),
        GroupDef(id: .l1, title: "L1", hint: buttonHint, inputs: [btn("btn_l1", 24, "l1")]),
        GroupDef(id: .r2, title: "R2", hint: "Analog trigger. As LFO rate: released = LFO stopped, fully pressed = fastest.",
                 inputs: [cc("r2_trigger", 10, "Pressure", axis)]),
        GroupDef(id: .r1, title: "R1", hint: buttonHint, inputs: [btn("btn_r1", 25, "r1")]),
        GroupDef(id: .dpad, title: "D-pad", hint: "↕ sends 8 steps (as Octave it shifts the pitch CV). ↔ picks the 16-slice section, or sends CC steps.",
                 inputs: [cc("dpad_vertical", 11, "Up / down", [.off, .cv, .octave]),
                          cc("dpad_horizontal", 13, "Left / right", [.off, .cv])]),
        GroupDef(id: .leftStick, title: "Left stick", hint: "X picks the slice inside the 16-slice section, or sends a CC. Springs back to the centre (64 ≈ 5 V).",
                 inputs: [cc("left_stick_x", 1, "X axis", axis), cc("left_stick_y", 2, "Y axis", axis),
                          btn("btn_l3", 20, "l3", "Click (L3)")]),
        GroupDef(id: .rightStick, title: "Right stick", hint: "Springs back to the centre (64 ≈ 5 V) when released.",
                 inputs: [cc("right_stick_x", 87, "X axis", axis), cc("right_stick_y", 88, "Y axis", axis),
                          btn("btn_r3", 21, "r3", "Click (R3)")]),
        GroupDef(id: .touchpad, title: "Touchpad", hint: "X also scrubs a playing loop, Y picks the quantize grid. Both keep their value when you lift the finger.",
                 inputs: [cc("touchpad_x", 30, "X position", axis), cc("touchpad_y", 31, "Y position", axis),
                          btn("btn_touchpad", 9, "touchpad", "Click")]),
        GroupDef(id: .create, title: "Create", hint: buttonHint, inputs: [btn("btn_create", 26, "create")]),
        GroupDef(id: .options, title: "Options", hint: buttonHint, inputs: [btn("btn_options", 27, "options")]),
        GroupDef(id: .ps, title: "PS button", hint: buttonHint, inputs: [btn("btn_ps", 28, "ps")]),
        GroupDef(id: .mic, title: "Mic button", hint: buttonHint + " (Read from the controller's HID report.)",
                 inputs: [btn("btn_mic", 29, "mic")]),
        GroupDef(id: .triangle, title: "△ Triangle", hint: buttonHint, inputs: [btn("btn_north", 22, "triangle")]),
        GroupDef(id: .circle, title: "○ Circle", hint: buttonHint, inputs: [btn("btn_east", 15, "circle")]),
        GroupDef(id: .cross, title: "✕ Cross", hint: buttonHint, inputs: [btn("btn_south", 14, "cross")]),
        GroupDef(id: .square, title: "□ Square", hint: buttonHint, inputs: [btn("btn_west", 23, "square")]),
        GroupDef(id: .motion, title: "Motion", hint: "Gyro / tilt as 14-bit values → 0–10 V in fine steps. Sent while motion control is on.",
                 inputs: [nrpn("tilt_x", 0, "Tilt X"), nrpn("tilt_y", 1, "Tilt Y"), nrpn("twist", 2, "Twist")]),
        GroupDef(id: .slice, title: "Grain slice", hint: "One 14-bit CV for 1024 slices: block of 128 (Create / Options by default) + section of 16 (D-pad ↔) + slice (left stick X).",
                 inputs: [nrpn("grain_slice", 3, "Slice 0–1023")]),
    ]

    static func group(_ id: GroupID) -> GroupDef { groups.first { $0.id == id }! }
    static let allInputs: [InputDef] = groups.flatMap(\.inputs)
    static func groupOf(_ input: InputDef) -> GroupDef { groups.first { $0.inputs.contains(input) }! }
    static func input(forButton b: String) -> InputDef? { allInputs.first { $0.button == b } }
}

// MARK: - Routes from other MIDI sources (pedals, EC4, 16n …)

struct ExternalRoute: Identifiable, Equatable {
    enum Kind: String, CaseIterable, Identifiable {
        case cv = "CC → CV", nrpn = "NRPN → CV", trigger = "CC → Trigger", note = "Note → Trigger"
        var id: String { rawValue }
    }
    var id = UUID()
    var kind: Kind = .cv
    var channel = 2
    var control = "1"        // number, or a name from CC_MAP / NRPN_MAP
    var port = 1
}

// MARK: - Everything the app edits

@MainActor @Observable
final class Mapping {
    // ER-301 routing (er301_routes.py)
    var channel = 2
    var roles: [String: Role] = [:]
    var ports: [String: Int] = [:]
    var pitchEnabled = true
    var pitchPort = 5
    var zeroNote = 60
    var semisPerStep = 12
    var lfoPort = 2
    var lfoMinHz = 0.25
    var lfoMaxHz = 40.0
    var lfoPulseMs = 5
    var triggersAsCV = true
    var externals: [ExternalRoute] = []
    var ccNames: [String: Int] = [:]
    var nrpnNames: [String: Int] = [:]

    // Button functions (controls.py)
    var funcs: [String: ButtonFunc] = Buttons.defaults
    var chords: [Chord] = Buttons.defaultChords
    var dpadHorizontal = "slice_section"
    var leftStickX = "slice_fine"
    var longPress = 1.0
    var controlsOnPi = true          // false: the Pi has no controls support yet (only routing is uploaded)

    init() { loadDefaults() }

    func loadDefaults() {
        roles = ["l2_trigger": .cv, "nrpn_grain_slice": .cv, "btn_south": .trigger,
                 "dpad_vertical": .octave, "r2_trigger": .lfoRate, "btn_west": .lfoReset]
        ports = ["l2_trigger": 8, "nrpn_grain_slice": 3, "btn_south": 2]
        pitchEnabled = true; pitchPort = 5; zeroNote = 60; semisPerStep = 12
        lfoPort = 2; lfoMinHz = 0.25; lfoMaxHz = 40; lfoPulseMs = 5
        triggersAsCV = true; channel = 2; externals = []
        funcs = Buttons.defaults; chords = Buttons.defaultChords
        dpadHorizontal = "slice_section"; leftStickX = "slice_fine"
    }

    // MARK: functions

    func fn(_ button: String) -> ButtonFunc { funcs[button] ?? ButtonFunc(tap: .none, hold: .none) }
    func setTap(_ a: Action, _ button: String) { var f = fn(button); f.tap = a; funcs[button] = f }
    func setHold(_ a: Action, _ button: String) { var f = fn(button); f.hold = a; funcs[button] = f }

    /// Does this input actually send its MIDI to the i2c2midi (given the button functions / modes)?
    func sendsMidi(_ i: InputDef) -> Bool {
        if let b = i.button { return fn(b).tap == .midi }
        if i.id == "dpad_horizontal" { return dpadHorizontal == "midi" }
        if i.id == "left_stick_x" { return leftStickX == "midi" }
        return true
    }

    // MARK: routing access

    func role(_ i: InputDef) -> Role { roles[i.id] ?? .off }
    /// The role that is in effect (a button that runs a function sends no MIDI).
    func effectiveRole(_ i: InputDef) -> Role { sendsMidi(i) ? role(i) : .off }
    func port(_ i: InputDef) -> Int { ports[i.id] ?? 1 }

    func setRole(_ r: Role, for i: InputDef) {
        if r.isExclusive {
            for (k, v) in roles where v == r { roles[k] = .off }
        }
        roles[i.id] = r
        if r.usesPort && ports[i.id] == nil { ports[i.id] = firstFreePort() }
    }

    func number(_ i: InputDef) -> Int {
        switch i.kind {
        case .cc: return ccNames[i.name ?? ""] ?? i.fallback
        case .nrpn: return nrpnNames[i.name ?? ""] ?? i.fallback
        case .note: return i.fallback
        }
    }

    func input(with role: Role) -> InputDef? { Catalog.allInputs.first { self.effectiveRole($0) == role } }

    func firstFreePort() -> Int {
        let used = Set(portMap.map(\.port))
        return (1...99).first { !used.contains($0) } ?? 1
    }

    var triggerPortName: String { triggersAsCV ? "SC.CV" : "SC.TR" }

    func routeToken(_ i: InputDef) -> String {
        switch effectiveRole(i) {
        case .off: return "—"
        case .cv: return "CV \(port(i))"
        case .trigger, .noteTrigger: return triggersAsCV ? "TRIG › CV \(port(i))" : "TR \(port(i))"
        case .lfoRate: return "LFO RATE"
        case .lfoReset: return "LFO RESET"
        case .octave: return "OCTAVE"
        }
    }

    /// What a control does, for the schematic: (text, kind) where kind picks the colour.
    enum TokenKind { case off, route(Role), function }
    func tokens(_ i: InputDef) -> [(String, TokenKind)] {
        if let b = i.button {
            let f = fn(b)
            var out: [(String, TokenKind)] = []
            if f.tap == .midi {
                let r = effectiveRole(i)
                out.append((routeToken(i), r == .off ? .off : .route(r)))
            } else {
                out.append((f.tap.token, f.tap == .none ? .off : .function))
            }
            if f.hold != .none { out.append(("HOLD " + f.hold.token, .function)) }
            return out
        }
        if i.id == "dpad_horizontal" && dpadHorizontal != "midi" { return [("SECTION ±", .function)] }
        if i.id == "left_stick_x" && leftStickX != "midi" { return [("SLICE", .function)] }
        let r = effectiveRole(i)
        return [(routeToken(i), r == .off ? .off : .route(r))]
    }

    // MARK: port overview

    struct PortUse: Identifiable {
        let kind: String; let port: Int; var users: [String]
        var refs: [PortRef] = []
        var id: String { "\(kind)\(port)" }
    }

    /// Where a port's signal comes from (the device tabs turn this into "16n · fader 3" etc.).
    enum PortRef: Hashable {
        case dualsense(label: String, inputID: String, trigger: Bool)
        case builtin(String)                       // pitch V/Oct, LFO
        case external(kind: String, channel: Int, control: String)
    }

    var portMap: [PortUse] {
        var map: [String: PortUse] = [:]
        func add(_ kind: String, _ port: Int, _ who: String, _ ref: PortRef) {
            let k = "\(kind)\(port)"
            map[k, default: PortUse(kind: kind, port: port, users: [])].users.append(who)
            map[k]?.refs.append(ref)
        }
        let tk = triggersAsCV ? "CV" : "TR"
        for i in Catalog.allInputs {
            let g = Catalog.groupOf(i)
            let who = g.inputs.count > 1 ? "\(g.title) \(i.label)" : g.title
            switch effectiveRole(i) {
            case .cv: add("CV", port(i), who, .dualsense(label: who, inputID: i.id, trigger: false))
            case .trigger, .noteTrigger: add(tk, port(i), who + " (trigger)", .dualsense(label: who, inputID: i.id, trigger: true))
            default: break
            }
        }
        if pitchEnabled { add("CV", pitchPort, "Pitch V/Oct", .builtin("Pitch V/Oct")) }
        if input(with: .lfoRate) != nil { add(tk, lfoPort, "LFO", .builtin("LFO")) }
        for e in externals {
            let who = "ch \(e.channel) \(e.kind == .note ? "note" : e.kind == .nrpn ? "NRPN" : "CC") \(e.control)"
            let k = e.kind == .note ? "note" : e.kind == .nrpn ? "nrpn" : e.kind == .trigger ? "trigger" : "cc"
            add(e.kind == .cv || e.kind == .nrpn ? "CV" : tk, e.port, who,
                .external(kind: k, channel: e.channel, control: e.control.trimmingCharacters(in: .whitespaces)))
        }
        return map.values.sorted { ($0.kind, $0.port) < ($1.kind, $1.port) }
    }

    // MARK: load from the Pi

    func apply(_ e: ExportDTO) {
        if let n = e.cc_names { ccNames = n }
        if let n = e.nrpn_names { nrpnNames = n }
        if let lp = e.long_press { longPress = lp }
        channel = e.channel ?? 2
        roles = [:]; ports = [:]; externals = []

        func resolve(_ c: StrOrInt, _ kind: SourceKind) -> Int? {
            switch c {
            case .int(let i): return i
            case .str(let s): return kind == .nrpn ? nrpnNames[s] : ccNames[s]
            }
        }
        func claim(_ entry: EntryDTO, _ kind: SourceKind, _ r: Role) -> Bool {
            guard entry.channel == channel, let num = resolve(entry.control, kind) else { return false }
            guard let i = Catalog.allInputs.first(where: {
                $0.kind == kind && number($0) == num && role($0) == .off && $0.roles.contains(r)
            }) else { return false }
            roles[i.id] = r
            if r.usesPort { ports[i.id] = entry.port }
            return true
        }
        for x in e.cv ?? [] where !claim(x, .cc, .cv) {
            externals.append(ExternalRoute(kind: .cv, channel: x.channel, control: x.control.text, port: x.port))
        }
        for x in e.nrpn_cv ?? [] where !claim(x, .nrpn, .cv) {
            externals.append(ExternalRoute(kind: .nrpn, channel: x.channel, control: x.control.text, port: x.port))
        }
        for x in e.triggers ?? [] where !claim(x, .cc, .trigger) {
            externals.append(ExternalRoute(kind: .trigger, channel: x.channel, control: x.control.text, port: x.port))
        }
        for x in e.note_triggers ?? [] {
            externals.append(ExternalRoute(kind: .note, channel: x.channel, control: x.control.text, port: x.port))
        }
        if let p = e.pitch {
            pitchEnabled = true
            pitchPort = p.port
            zeroNote = p.zero_note ?? 60
            semisPerStep = p.semitones_per_step ?? 12
            if let oc = p.octave_control { _ = claim(EntryDTO(channel: channel, control: oc, port: 0), .cc, .octave) }
        } else {
            pitchEnabled = false
        }
        if let l = e.lfo {
            lfoPort = l.port
            lfoMinHz = l.min_hz ?? 0.25
            lfoMaxHz = l.max_hz ?? 40
            lfoPulseMs = l.pulse_ms ?? 5
            if let rc = l.rate_control { _ = claim(EntryDTO(channel: channel, control: rc, port: 0), .cc, .lfoRate) }
            if let rs = l.reset_control { _ = claim(EntryDTO(channel: channel, control: rs, port: 0), .cc, .lfoReset) }
        }
        triggersAsCV = e.triggers_as_cv ?? true

        if let c = e.controls {
            controlsOnPi = true
            var f = Buttons.defaults
            for (name, v) in c.buttons {
                f[name] = ButtonFunc(tap: Action(rawValue: v.tap) ?? .none,
                                     hold: v.hold.flatMap { Action(rawValue: $0) } ?? .none)
            }
            funcs = f
            chords = c.chords.compactMap { x in
                guard x.count == 3, let a = Action(rawValue: x[2]) else { return nil }
                return Chord(a: x[0], b: x[1], action: a)
            }
            dpadHorizontal = c.dpad_horizontal
            leftStickX = c.left_stick_x
        } else {
            controlsOnPi = e.controls_error != nil     // file broken: still supported, but needs fixing
            funcs = Buttons.defaults; chords = Buttons.defaultChords
            dpadHorizontal = "slice_section"; leftStickX = "slice_fine"
        }
    }

    // MARK: write er301_routes.py

    private func ctl(_ i: InputDef) -> String {
        let names = i.kind == .nrpn ? nrpnNames : ccNames
        if let n = i.name, names[n] != nil { return "'\(n)'" }
        return String(number(i))
    }

    private func extLiteral(_ s: String) -> String {
        let t = s.trimmingCharacters(in: .whitespaces)
        if let n = Int(t) { return String(n) }
        let clean = t.filter { $0.isLetter || $0.isNumber || $0 == "_" }
        return "'\(clean.isEmpty ? "unknown" : clean)'"
    }

    private func line(_ body: String, _ comment: String) -> String {
        let pad = max(1, 30 - body.count)
        return "    " + body + String(repeating: " ", count: pad) + "# " + comment + "\n"
    }

    private func fmt(_ d: Double) -> String { String(format: "%g", d) }

    func pythonSource(notes: [String: String] = [:]) -> String {
        func noted(_ c: String, _ key: String) -> String {
            guard let n = notes[key] else { return c }
            return c + " · " + n.replacingOccurrences(of: "\n", with: " ")
        }
        var s = """
        \"\"\"
        ER-301 routing inside the i2c2midi (DualSense → ER-301 directly, no Teletype script).

        Written by Kontrolleur on the Mac. You can still edit it by hand; the app reads it back.
        Saving makes the MIDI hub (midi-passthrough service) send it to the i2c2midi within a
        second; the i2c2midi keeps it across power cycles. Check by hand:

            python3 ~/dualsense-eurorack-midi/scripts/er301_push.py

        Ports are the ER-301 SC.CV / SC.TR numbers (1–99) chosen as the source of a unit's input.
        Entries are (control, port) on CHANNEL, or (channel, control, port) for another channel.
        \"\"\"

        CHANNEL = \(channel)            # DualSense instrument channel


        """
        let tp = triggerPortName
        func section(_ header: String, _ name: String, _ rows: [String]) {
            s += header + "\n\(name) = [\n" + rows.joined() + "]\n\n"
        }
        func title(_ i: InputDef) -> String {
            let g = Catalog.groupOf(i)
            return g.inputs.count > 1 ? "\(g.title) \(i.label.lowercased())" : g.title
        }

        var rows: [String] = []
        for i in Catalog.allInputs where i.kind == .cc && effectiveRole(i) == .cv {
            rows.append(line("(\(ctl(i)), \(port(i))),", noted("\(title(i)) → SC.CV \(port(i))", "ds:" + i.id)))
        }
        for e in externals where e.kind == .cv {
            rows.append(line("(\(e.channel), \(extLiteral(e.control)), \(e.port)),", noted("ch \(e.channel) CC \(e.control) → SC.CV \(e.port)", "ext:\(e.channel):\(e.control.trimmingCharacters(in: .whitespaces))")))
        }
        section("# CC → SC.CV   (0–127 → 0–10 V)", "CV", rows)

        rows = []
        for i in Catalog.allInputs where i.kind == .nrpn && effectiveRole(i) == .cv {
            rows.append(line("(\(ctl(i)), \(port(i))),", noted("\(title(i)) → SC.CV \(port(i))", "ds:" + i.id)))
        }
        for e in externals where e.kind == .nrpn {
            rows.append(line("(\(e.channel), \(extLiteral(e.control)), \(e.port)),", noted("ch \(e.channel) NRPN \(e.control) → SC.CV \(e.port)", "ext:\(e.channel):\(e.control.trimmingCharacters(in: .whitespaces))")))
        }
        section("# 14-bit NRPN → SC.CV   (0–16383 → 0–10 V, full resolution)", "NRPN_CV", rows)

        rows = []
        for i in Catalog.allInputs where i.kind == .cc && effectiveRole(i) == .trigger {
            rows.append(line("(\(ctl(i)), \(port(i))),", noted("\(title(i)) → \(tp) \(port(i))", "ds:" + i.id)))
        }
        for e in externals where e.kind == .trigger {
            rows.append(line("(\(e.channel), \(extLiteral(e.control)), \(e.port)),", noted("ch \(e.channel) CC \(e.control) → \(tp) \(e.port)", "ext:\(e.channel):\(e.control.trimmingCharacters(in: .whitespaces))")))
        }
        section("# CC as gate (≥ 64 = high) → trigger port", "TRIGGERS", rows)

        rows = []
        for e in externals where e.kind == .note {
            rows.append(line("(\(e.channel), \(extLiteral(e.control)), \(e.port)),", noted("ch \(e.channel) note \(e.control) → \(tp) \(e.port)", "ext:\(e.channel):\(e.control.trimmingCharacters(in: .whitespaces))")))
        }
        section("# Note held → trigger port. These notes do not change the pitch.", "NOTE_TRIGGERS", rows)

        s += "# Notes on CHANNEL → V/Oct. The octave control adds steps of semitones_per_step.\n"
        if pitchEnabled {
            let oc = input(with: .octave).map { ctl($0) } ?? "None"
            s += "PITCH = dict(port=\(pitchPort), zero_note=\(zeroNote), octave_control=\(oc), semitones_per_step=\(semisPerStep))\n\n"
        } else {
            s += "PITCH = None\n\n"
        }

        s += "# Trigger LFO: rate control 0 = stopped, 1–127 = min_hz … max_hz (exponential).\n"
        if let rate = input(with: .lfoRate) {
            let reset = input(with: .lfoReset).map { ctl($0) } ?? "None"
            s += "LFO = dict(port=\(lfoPort), rate_control=\(ctl(rate)), reset_control=\(reset),\n"
            s += "           min_hz=\(fmt(lfoMinHz)), max_hz=\(fmt(lfoMaxHz)), pulse_ms=\(lfoPulseMs))\n\n"
        } else {
            s += "LFO = None\n\n"
        }

        s += "# True: trigger ports are SC.CV ports (10 V pulse / gate). False: SC.TR ports.\n"
        s += "TRIGGERS_AS_CV = \(triggersAsCV ? "True" : "False")\n"
        return s
    }

    // MARK: write controls.py

    func controlsSource() -> String {
        var s = """
        \"\"\"
        What the DualSense buttons do. Read by the DualSense script; changes apply within a second.
        Written by Kontrolleur on the Mac. You can still edit it by hand; the app reads it back.

        tap  = short press   (loop actions fire when you let go)
        hold = held for LONG_PRESS_DURATION (mappings.py) or longer, decided when you let go
        'midi' = the button sends its own CC (127 pressed / 0 released): route it to the ER-301
                 in er301_routes.py. Buttons: cross circle triangle square l1 r1 l3 r3 create
                 options ps mic touchpad (= touchpad click).
        Actions: none midi loop_record loop_play loop_clear motion block_up block_down block_reset
                 section_up section_down strum freeze_left freeze_right quantize
        \"\"\"

        BUTTONS = {

        """
        for (name, _) in Buttons.all {
            let f = fn(name)
            let key = "'\(name)':"
            var body = "dict(tap='\(f.tap.rawValue)'"
            if f.hold != .none { body += ", hold='\(f.hold.rawValue)'" }
            body += "),"
            s += "    " + key + String(repeating: " ", count: max(1, 12 - key.count)) + body + "\n"
        }
        s += "}\n\n# Two buttons pressed together. The buttons' own release actions are skipped then.\nCHORDS = [\n"
        for c in chords { s += "    ('\(c.a)', '\(c.b)', '\(c.action.rawValue)'),\n" }
        s += "]\n\n"
        s += "DPAD_HORIZONTAL = '\(dpadHorizontal)'   # 'slice_section' (16-slice sections) or 'midi' (CC steps)\n"
        s += "LEFT_STICK_X = '\(leftStickX)'         # 'slice_fine' (slice within the section) or 'midi' (CC)\n"
        return s
    }

    // MARK: 16n fader routes (CC → CV routes on the fader's USB channel / CC)

    func faderPort(channel: Int, cc: Int) -> Int? {
        externals.first { $0.kind == .cv && $0.channel == channel && Int($0.control.trimmingCharacters(in: .whitespaces)) == cc }?.port
    }

    func setFaderPort(channel: Int, cc: Int, port: Int?) {
        if let idx = externals.firstIndex(where: { $0.kind == .cv && $0.channel == channel && Int($0.control.trimmingCharacters(in: .whitespaces)) == cc }) {
            if let port { externals[idx].port = port } else { externals.remove(at: idx) }
        } else if let port {
            externals.append(ExternalRoute(kind: .cv, channel: channel, control: String(cc), port: port))
        }
    }

    /// Things the Pi would reject, shown before uploading.
    var problems: [String] {
        var p: [String] = []
        if lfoMinHz < 0.01 || lfoMaxHz > 163.83 || lfoMinHz > lfoMaxHz { p.append("LFO range must be 0.01 ≤ min ≤ max ≤ 163.83 Hz.") }
        for c in chords where c.a == c.b {
            p.append("Combo \(Buttons.label(c.a)) + \(Buttons.label(c.b)) needs two different buttons.")
        }
        for e in externals {
            let t = e.control.trimmingCharacters(in: .whitespaces)
            if let n = Int(t) {
                let maxN = e.kind == .nrpn ? 16383 : 127
                if n < 0 || n > maxN { p.append("\(e.kind.rawValue) ch \(e.channel): \(n) is out of range 0–\(maxN).") }
            } else {
                let names = e.kind == .nrpn ? nrpnNames : ccNames
                if e.kind == .note || (!names.isEmpty && names[t] == nil) {
                    p.append("\(e.kind.rawValue) ch \(e.channel): “\(t)” is not a known \(e.kind == .note ? "note number" : "control").")
                }
            }
        }
        return p
    }
}

// MARK: - JSON from scripts/er301_routes_tool.py on the Pi

enum StrOrInt: Decodable, Equatable {
    case str(String), int(Int)
    init(from decoder: Decoder) throws {
        let c = try decoder.singleValueContainer()
        if let i = try? c.decode(Int.self) { self = .int(i) } else { self = .str(try c.decode(String.self)) }
    }
    var text: String {
        switch self {
        case .str(let s): return s
        case .int(let i): return String(i)
        }
    }
}

struct EntryDTO: Decodable { let channel: Int; let control: StrOrInt; let port: Int }
struct PitchDTO: Decodable { let port: Int; let zero_note: Int?; let octave_control: StrOrInt?; let semitones_per_step: Int? }
struct LfoDTO: Decodable {
    let port: Int; let rate_control: StrOrInt?; let reset_control: StrOrInt?
    let min_hz: Double?; let max_hz: Double?; let pulse_ms: Int?
}
struct ButtonDTO: Decodable { let tap: String; let hold: String? }
struct ControlsDTO: Decodable {
    let buttons: [String: ButtonDTO]
    let chords: [[String]]
    let dpad_horizontal: String
    let left_stick_x: String
}
struct ExportDTO: Decodable {
    let ok: Bool; let error: String?
    let cc_names: [String: Int]?; let nrpn_names: [String: Int]?
    let long_press: Double?
    let channel: Int?
    let cv: [EntryDTO]?; let nrpn_cv: [EntryDTO]?; let triggers: [EntryDTO]?; let note_triggers: [EntryDTO]?
    let pitch: PitchDTO?; let lfo: LfoDTO?; let triggers_as_cv: Bool?
    let controls: ControlsDTO?; let controls_error: String?
}
struct InstallDTO: Decodable { let ok: Bool; let summary: String?; let backup: String?; let hub: String?; let error: String? }

// MARK: - SSH to the Pi

struct PiClient {
    var host: String
    var identity: String
    var repo: String

    enum Failure: LocalizedError {
        case ssh(String), output(String)
        var errorDescription: String? {
            switch self {
            case .ssh(let s): return s
            case .output(let s): return s
            }
        }
    }

    private func run(_ remote: String, input: String? = nil) async throws -> String {
        let host = host, identity = identity
        return try await withCheckedThrowingContinuation { cont in
            DispatchQueue.global(qos: .userInitiated).async {
                let p = Process()
                p.executableURL = URL(fileURLWithPath: "/usr/bin/ssh")
                var args = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=accept-new"]
                let id = identity.trimmingCharacters(in: .whitespaces)
                if !id.isEmpty { args += ["-o", "IdentitiesOnly=yes", "-i", (id as NSString).expandingTildeInPath] }
                args += [host, remote]
                p.arguments = args
                let out = Pipe(), err = Pipe(), inp = Pipe()
                p.standardOutput = out; p.standardError = err; p.standardInput = inp
                do { try p.run() } catch { cont.resume(throwing: error); return }
                if let input, let d = input.data(using: .utf8) { inp.fileHandleForWriting.write(d) }
                try? inp.fileHandleForWriting.close()
                let o = out.fileHandleForReading.readDataToEndOfFile()
                let e = err.fileHandleForReading.readDataToEndOfFile()
                p.waitUntilExit()
                let so = String(decoding: o, as: UTF8.self), se = String(decoding: e, as: UTF8.self)
                if p.terminationStatus == 255 {
                    cont.resume(throwing: Failure.ssh(se.trimmingCharacters(in: .whitespacesAndNewlines)))
                } else {
                    cont.resume(returning: so.isEmpty ? se : so)
                }
            }
        }
    }

    private func json<T: Decodable>(_ text: String) throws -> T {
        guard let line = text.split(separator: "\n").last(where: { $0.hasPrefix("{") }),
              let data = line.data(using: .utf8) else {
            throw Failure.output(text.isEmpty ? "No answer from the Pi helper." : text.trimmingCharacters(in: .whitespacesAndNewlines))
        }
        return try JSONDecoder().decode(T.self, from: data)
    }

    private var tool: String { "python3 \(repo)/scripts/er301_routes_tool.py" }

    func runRemote(_ command: String, input: String? = nil) async throws -> String { try await run(command, input: input) }

    func export() async throws -> ExportDTO { try json(try await run("\(tool) export")) }
    func install(_ source: String) async throws -> InstallDTO { try json(try await run("\(tool) install", input: source)) }
    func installControls(_ source: String) async throws -> InstallDTO {
        try json(try await run("\(tool) install-controls", input: source))
    }
}
