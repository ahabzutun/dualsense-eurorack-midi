import SwiftUI
import AppKit
import Observation

/// Colours, fonts and text size of the app. Saved in the app's preferences; presets included.
@Observable
final class Appearance {
    nonisolated(unsafe) static let shared = Appearance()

    struct RGBA: Codable, Equatable {
        var r, g, b, a: Double
        init(_ r: Double, _ g: Double, _ b: Double, _ a: Double = 1) { self.r = r; self.g = g; self.b = b; self.a = a }
        init(_ c: Color) {
            let n = NSColor(c).usingColorSpace(.sRGB) ?? .gray
            r = n.redComponent; g = n.greenComponent; b = n.blueComponent; a = n.alphaComponent
        }
        var color: Color { Color(.sRGB, red: r, green: g, blue: b, opacity: a) }
        var luminance: Double { 0.2126 * r + 0.7152 * g + 0.0722 * b }
    }

    enum Slot: String, CaseIterable, Codable, CodingKeyRepresentable, Identifiable {
        case bg, panel, card, line, ink, dim, body, cv, trig, special, octave, function, note
        var id: String { rawValue }
        var title: String {
            switch self {
            case .bg: "Background"
            case .panel: "Side panel & bars"
            case .card: "Cards"
            case .line: "Dividers & borders"
            case .ink: "Text"
            case .dim: "Secondary text"
            case .body: "Drawing lines"
            case .cv: "CV / main accent"
            case .trig: "Triggers"
            case .special: "LFO & special"
            case .octave: "Octave / pitch"
            case .function: "Button functions"
            case .note: "Notes"
            }
        }
        var group: String {
            switch self {
            case .bg, .panel, .card, .line: "Surfaces"
            case .ink, .dim, .body: "Text & lines"
            default: "Accents"
            }
        }
    }

    struct Look: Codable, Equatable {
        var colors: [Slot: RGBA]
        var uiFont = "System"
        var monoFont = "System Mono"
        var titleFont = "System Rounded"
        var scale = 1.0
    }

    // MARK: presets

    static let presets: [(String, Look)] = [
        ("Blueprint (dark)", Look(colors: [
            .bg: RGBA(0.047, 0.059, 0.078), .panel: RGBA(0.075, 0.090, 0.114), .card: RGBA(0.102, 0.122, 0.153),
            .line: RGBA(1, 1, 1, 0.09), .ink: RGBA(0.88, 0.92, 0.97), .dim: RGBA(1, 1, 1, 0.42),
            .body: RGBA(0.62, 0.74, 0.86), .cv: RGBA(0.27, 0.86, 0.96), .trig: RGBA(1.00, 0.63, 0.28),
            .special: RGBA(0.55, 0.94, 0.56), .octave: RGBA(0.76, 0.62, 1.00), .function: RGBA(0.98, 0.84, 0.36),
            .note: RGBA(0.98, 0.86, 0.55)])),
        ("Paper (light)", Look(colors: [
            .bg: RGBA(0.95, 0.94, 0.91), .panel: RGBA(0.91, 0.90, 0.87), .card: RGBA(0.99, 0.98, 0.96),
            .line: RGBA(0, 0, 0, 0.12), .ink: RGBA(0.11, 0.12, 0.14), .dim: RGBA(0, 0, 0, 0.48),
            .body: RGBA(0.27, 0.36, 0.48), .cv: RGBA(0.00, 0.47, 0.66), .trig: RGBA(0.84, 0.38, 0.05),
            .special: RGBA(0.13, 0.55, 0.20), .octave: RGBA(0.45, 0.30, 0.78), .function: RGBA(0.67, 0.50, 0.00),
            .note: RGBA(0.62, 0.42, 0.05)], titleFont: "System Serif")),
        ("Amber terminal", Look(colors: [
            .bg: RGBA(0.04, 0.03, 0.01), .panel: RGBA(0.07, 0.05, 0.02), .card: RGBA(0.10, 0.07, 0.03),
            .line: RGBA(1.0, 0.70, 0.25, 0.14), .ink: RGBA(1.00, 0.76, 0.33), .dim: RGBA(1.0, 0.70, 0.30, 0.50),
            .body: RGBA(0.90, 0.60, 0.22), .cv: RGBA(1.00, 0.80, 0.35), .trig: RGBA(1.00, 0.48, 0.20),
            .special: RGBA(0.95, 0.90, 0.55), .octave: RGBA(1.00, 0.62, 0.45), .function: RGBA(1.00, 0.88, 0.50),
            .note: RGBA(1.00, 0.93, 0.70)], uiFont: "System Mono", monoFont: "System Mono", titleFont: "System Mono")),
        ("High contrast", Look(colors: [
            .bg: RGBA(0, 0, 0), .panel: RGBA(0.05, 0.05, 0.05), .card: RGBA(0.10, 0.10, 0.10),
            .line: RGBA(1, 1, 1, 0.35), .ink: RGBA(1, 1, 1), .dim: RGBA(1, 1, 1, 0.75),
            .body: RGBA(1, 1, 1), .cv: RGBA(0.20, 0.90, 1.00), .trig: RGBA(1.00, 0.60, 0.10),
            .special: RGBA(0.40, 1.00, 0.40), .octave: RGBA(0.85, 0.65, 1.00), .function: RGBA(1.00, 0.95, 0.20),
            .note: RGBA(1.00, 1.00, 0.55)], scale: 1.1)),
        ("Eurorack panel", Look(colors: [
            .bg: RGBA(0.16, 0.16, 0.17), .panel: RGBA(0.20, 0.20, 0.21), .card: RGBA(0.25, 0.25, 0.26),
            .line: RGBA(1, 1, 1, 0.10), .ink: RGBA(0.93, 0.93, 0.91), .dim: RGBA(1, 1, 1, 0.50),
            .body: RGBA(0.80, 0.80, 0.78), .cv: RGBA(0.95, 0.35, 0.30), .trig: RGBA(1.00, 0.78, 0.20),
            .special: RGBA(0.35, 0.75, 0.95), .octave: RGBA(0.70, 0.55, 0.95), .function: RGBA(0.98, 0.90, 0.60),
            .note: RGBA(1.00, 0.85, 0.60)])),
    ]

    var data: Look { didSet { if data != oldValue { revision += 1; if !snapshotOnly { save() } } } }
    var revision = 0
    @ObservationIgnored private var snapshotOnly = false

    init() {
        if let d = UserDefaults.standard.data(forKey: "appearance"), let v = try? JSONDecoder().decode(Look.self, from: d) {
            data = v
        } else {
            data = Appearance.presets[0].1
        }
        // DSM_PRESET=<index>: try a preset for a UI snapshot without saving it
        if let i = Int(ProcessInfo.processInfo.environment["DSM_PRESET"] ?? ""), Appearance.presets.indices.contains(i) {
            snapshotOnly = true
            data = Appearance.presets[i].1
        }
        // DSM_SCALE=1.35 / DSM_FONT=<family>: text size and font for UI snapshots, not saved
        let env = ProcessInfo.processInfo.environment
        if let s = Double(env["DSM_SCALE"] ?? "") { snapshotOnly = true; data.scale = s }
        if let f = env["DSM_FONT"] { snapshotOnly = true; data.monoFont = f; data.uiFont = f; data.titleFont = f }
    }

    private func save() {
        if let d = try? JSONEncoder().encode(data) { UserDefaults.standard.set(d, forKey: "appearance") }
    }

    func color(_ s: Slot) -> Color { (data.colors[s] ?? Appearance.presets[0].1.colors[s] ?? RGBA(0.5, 0.5, 0.5)).color }

    func binding(_ s: Slot) -> Binding<Color> {
        Binding(get: { self.color(s) }, set: { self.data.colors[s] = RGBA($0) })
    }

    var isDark: Bool { (data.colors[.bg] ?? RGBA(0, 0, 0)).luminance < 0.5 }
    var colorScheme: ColorScheme { isDark ? .dark : .light }

    /// Background for number / text fields: a bit darker (dark themes) or lighter (light themes) than the card.
    var fieldFill: Color { isDark ? Color.black.opacity(0.35) : Color.white.opacity(0.75) }

    // MARK: fonts

    enum FontRole { case ui, mono, title }

    static let builtInFonts = ["System", "System Mono", "System Rounded", "System Serif"]
    static var installedFamilies: [String] { NSFontManager.shared.availableFontFamilies.sorted() }

    func font(_ size: CGFloat, _ weight: Font.Weight, _ role: FontRole) -> Font {
        let s = size * data.scale
        let family = role == .mono ? data.monoFont : (role == .title ? data.titleFont : data.uiFont)
        switch family {
        case "System": return .system(size: s, weight: weight, design: .default)
        case "System Mono": return .system(size: s, weight: weight, design: .monospaced)
        case "System Rounded": return .system(size: s, weight: weight, design: .rounded)
        case "System Serif": return .system(size: s, weight: weight, design: .serif)
        default: return Font.custom(family, size: s).weight(weight)
        }
    }
}

// MARK: - Settings panel

@MainActor
struct AppearancePanel: View {
    @State private var ap = Appearance.shared
    @State private var families: [String] = []

    var body: some View {
        @Bindable var a = ap
        ScrollView {
            VStack(alignment: .leading, spacing: 14) {
                Text("Appearance").font(.title2.bold())
                HStack {
                    Text("Preset")
                    Spacer()
                    Menu("Choose …") {
                        ForEach(Appearance.presets, id: \.0) { p in
                            Button(p.0) { ap.data = p.1 }
                        }
                    }
                    .fixedSize()
                }
                ForEach(["Surfaces", "Text & lines", "Accents"], id: \.self) { g in
                    GroupBox(g) {
                        VStack(spacing: 6) {
                            ForEach(Appearance.Slot.allCases.filter { $0.group == g }) { s in
                                ColorPicker(s.title, selection: ap.binding(s), supportsOpacity: true)
                            }
                        }
                        .padding(4)
                    }
                }
                GroupBox("Fonts") {
                    VStack(spacing: 8) {
                        fontPicker("Interface", $a.data.uiFont)
                        fontPicker("Labels & numbers", $a.data.monoFont)
                        fontPicker("Titles", $a.data.titleFont)
                        HStack {
                            Text("Text size")
                            Slider(value: $a.data.scale, in: 0.8...1.35, step: 0.05)
                            Text("\(Int((ap.data.scale * 100).rounded()))%").monospacedDigit().frame(width: 44, alignment: .trailing)
                        }
                    }
                    .padding(4)
                }
                Text("Changes apply right away and are kept on this computer. The drawings make room for bigger text and fit themselves to the window: zoom in there (pinch, scroll wheel, ⌘+) to see them larger.")
                    .font(.caption).foregroundStyle(.secondary)
                HStack {
                    Spacer()
                    Button("Reset to Blueprint") { ap.data = Appearance.presets[0].1 }
                }
            }
            .padding(18)
        }
        .frame(width: 380, height: 640)
        .onAppear { if families.isEmpty { families = Appearance.installedFamilies } }
    }

    private func fontPicker(_ title: String, _ value: Binding<String>) -> some View {
        Picker(title, selection: value) {
            ForEach(Appearance.builtInFonts, id: \.self) { Text($0).tag($0) }
            Divider()
            ForEach(families, id: \.self) { Text($0).tag($0) }
        }
    }
}
