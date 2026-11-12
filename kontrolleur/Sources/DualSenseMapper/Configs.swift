import SwiftUI
import AppKit
import UniformTypeIdentifiers

// MARK: - DualSense mapping as plain data

struct MappingSnapshot: Codable, Equatable {
    struct Ext: Codable, Equatable { var kind: String; var channel: Int; var control: String; var port: Int }
    struct Fn: Codable, Equatable { var tap: String; var hold: String }
    struct Ch: Codable, Equatable { var a: String; var b: String; var action: String }

    var channel: Int
    var roles: [String: String]
    var ports: [String: Int]
    var pitchEnabled: Bool
    var pitchPort: Int
    var zeroNote: Int
    var semisPerStep: Int
    var lfoPort: Int
    var lfoMinHz: Double
    var lfoMaxHz: Double
    var lfoPulseMs: Int
    var triggersAsCV: Bool
    var externals: [Ext]
    var funcs: [String: Fn]
    var chords: [Ch]
    var dpadHorizontal: String
    var leftStickX: String
    var longPress: Double
}

extension Mapping {
    var snapshot: MappingSnapshot {
        MappingSnapshot(
            channel: channel,
            roles: roles.mapValues(\.rawValue),
            ports: ports,
            pitchEnabled: pitchEnabled, pitchPort: pitchPort, zeroNote: zeroNote, semisPerStep: semisPerStep,
            lfoPort: lfoPort, lfoMinHz: lfoMinHz, lfoMaxHz: lfoMaxHz, lfoPulseMs: lfoPulseMs,
            triggersAsCV: triggersAsCV,
            externals: externals.map { .init(kind: $0.kind.rawValue, channel: $0.channel, control: $0.control, port: $0.port) },
            funcs: funcs.mapValues { .init(tap: $0.tap.rawValue, hold: $0.hold.rawValue) },
            chords: chords.map { .init(a: $0.a, b: $0.b, action: $0.action.rawValue) },
            dpadHorizontal: dpadHorizontal, leftStickX: leftStickX, longPress: longPress)
    }

    func restore(_ s: MappingSnapshot) {
        channel = s.channel
        roles = s.roles.compactMapValues { Role(rawValue: $0) }
        ports = s.ports
        pitchEnabled = s.pitchEnabled; pitchPort = s.pitchPort; zeroNote = s.zeroNote; semisPerStep = s.semisPerStep
        lfoPort = s.lfoPort; lfoMinHz = s.lfoMinHz; lfoMaxHz = s.lfoMaxHz; lfoPulseMs = s.lfoPulseMs
        triggersAsCV = s.triggersAsCV
        externals = s.externals.compactMap { e in
            ExternalRoute.Kind(rawValue: e.kind).map { ExternalRoute(kind: $0, channel: e.channel, control: e.control, port: e.port) }
        }
        var f = Buttons.defaults
        for (k, v) in s.funcs {
            if let t = Action(rawValue: v.tap), let h = Action(rawValue: v.hold) { f[k] = ButtonFunc(tap: t, hold: h) }
        }
        funcs = f
        chords = s.chords.compactMap { c in Action(rawValue: c.action).map { Chord(a: c.a, b: c.b, action: $0) } }
        dpadHorizontal = s.dpadHorizontal
        leftStickX = s.leftStickX
        longPress = s.longPress
    }
}

// MARK: - The file

struct ConfigFile: Codable {
    struct Fader16: Codable { var faders: [Fader]; var options: FaderOptions }
    struct EC4Data: Codable { var mem: String; var version: Double; var setups: [EC4Setup] }

    var format = "dualsense-mapper-configuration"
    var version = 1
    var saved: Date
    var mapping: MappingSnapshot?
    var faderbank: Fader16?
    var pedals: [PedalInput]?
    var ec4: EC4Data?
    var notes: [String: String]?

    enum Part: String, CaseIterable, Identifiable {
        case dualsense = "DualSense routing & button functions"
        case faderbank = "16n faders"
        case pedals = "MPC-20 pedals"
        case ec4 = "EC4 setups"
        case notes = "Notes"
        var id: String { rawValue }
    }

    func has(_ p: Part) -> Bool {
        switch p {
        case .dualsense: mapping != nil
        case .faderbank: faderbank != nil
        case .pedals: pedals != nil
        case .ec4: ec4 != nil
        case .notes: !(notes ?? [:]).isEmpty
        }
    }
}

enum ConfigLibrary {
    static let type = UTType(filenameExtension: "dsmconfig", conformingTo: .json) ?? .json

    static var folder: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let dir = docs.appendingPathComponent("Kontrolleur", isDirectory: true)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    /// Recently used files (remembered) plus the ones in the default folder, newest first.
    static func recent(limit: Int = 10) -> [URL] {
        var urls = (UserDefaults.standard.stringArray(forKey: "recentConfigs") ?? []).map { URL(fileURLWithPath: $0) }
        let old = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("DualSense Mapper")
        for dir in [folder, old] {
            if let inFolder = try? FileManager.default.contentsOfDirectory(at: dir, includingPropertiesForKeys: [.contentModificationDateKey]) {
                urls += inFolder.filter { $0.pathExtension == "dsmconfig" }
            }
        }
        var seen = Set<String>()
        let existing = urls.filter { FileManager.default.fileExists(atPath: $0.path) && seen.insert($0.standardizedFileURL.path).inserted }
        func date(_ u: URL) -> Date { (try? u.resourceValues(forKeys: [.contentModificationDateKey]).contentModificationDate) ?? .distantPast }
        return Array(existing.sorted { date($0) > date($1) }.prefix(limit))
    }

    static func remember(_ url: URL) {
        var list = UserDefaults.standard.stringArray(forKey: "recentConfigs") ?? []
        list.removeAll { $0 == url.path }
        list.insert(url.path, at: 0)
        UserDefaults.standard.set(Array(list.prefix(20)), forKey: "recentConfigs")
    }

    static func write(_ c: ConfigFile, to url: URL) throws {
        let enc = JSONEncoder()
        enc.outputFormatting = [.prettyPrinted, .sortedKeys]
        enc.dateEncodingStrategy = .iso8601
        try enc.encode(c).write(to: url, options: .atomic)
        remember(url)
    }

    static func read(_ url: URL) throws -> ConfigFile {
        let dec = JSONDecoder()
        dec.dateDecodingStrategy = .iso8601
        let c = try dec.decode(ConfigFile.self, from: Data(contentsOf: url))
        guard c.format == "dualsense-mapper-configuration" else {
            throw NSError(domain: "DualSenseMapper", code: 1, userInfo: [NSLocalizedDescriptionKey: "This is not a Kontrolleur configuration."])
        }
        remember(url)
        return c
    }

    @MainActor static func askSave(suggested: String, _ done: @escaping (URL) -> Void) {
        let p = NSSavePanel()
        p.title = "Save configuration"
        p.nameFieldStringValue = suggested
        p.directoryURL = folder
        p.allowedContentTypes = [type]
        p.canCreateDirectories = true
        p.begin { r in if r == .OK, let u = p.url { done(u) } }
    }

    @MainActor static func askOpen(_ done: @escaping (URL) -> Void) {
        let p = NSOpenPanel()
        p.title = "Open configuration"
        p.directoryURL = folder
        p.allowedContentTypes = [type, .json]
        p.allowsMultipleSelection = false
        p.begin { r in if r == .OK, let u = p.url { done(u) } }
    }
}

// MARK: - Choosing what to load

struct PendingConfig: Identifiable {
    let id = UUID()
    let url: URL
    let file: ConfigFile
}

struct ConfigLoadSheet: View {
    let pending: PendingConfig
    let apply: (Set<ConfigFile.Part>) -> Void
    @Environment(\.dismiss) private var dismiss
    @State private var parts: Set<ConfigFile.Part> = []

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            Text("Load configuration").font(.title2.bold())
            VStack(alignment: .leading, spacing: 2) {
                Text(pending.url.deletingPathExtension().lastPathComponent).font(.headline)
                Text("Saved \(pending.file.saved.formatted(date: .abbreviated, time: .shortened))")
                    .font(.caption).foregroundStyle(.secondary)
            }
            VStack(alignment: .leading, spacing: 8) {
                ForEach(ConfigFile.Part.allCases) { p in
                    Toggle(isOn: Binding(get: { parts.contains(p) }, set: { on in if on { parts.insert(p) } else { parts.remove(p) } })) {
                        HStack {
                            Text(p.rawValue)
                            if !pending.file.has(p) { Text("not in this file").font(.caption).foregroundStyle(.secondary) }
                        }
                    }
                    .disabled(!pending.file.has(p))
                }
            }
            Text("Loading only changes the app. Send it to the devices with “Upload” on each tab. Notes are replaced by the ones in the file.")
                .font(.caption).foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)
            HStack {
                Spacer()
                Button("Cancel", role: .cancel) { dismiss() }.keyboardShortcut(.cancelAction)
                Button("Load") { apply(parts); dismiss() }
                    .keyboardShortcut(.defaultAction)
                    .disabled(parts.isEmpty)
            }
        }
        .padding(22)
        .frame(width: 440)
        .onAppear { parts = Set(ConfigFile.Part.allCases.filter { pending.file.has($0) }) }
    }
}
