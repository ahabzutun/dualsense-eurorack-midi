import SwiftUI
import Observation

/// Free-text notes per control ("what is this routed for?"), kept on this Mac.
/// Notes of controls with an ER-301 route also end up as comments in er301_routes.py.
@MainActor @Observable
final class NotesStore {
    var notes: [String: String] = [:] { didSet { save() } }

    init() { load() }

    // keys
    static func ds(_ inputID: String) -> String { "ds:" + inputID }
    static func fader(_ i: Int) -> String { "16n:\(i)" }
    static func pedal(_ input: Int, _ slot: Int) -> String { "mpc:\(input):\(slot)" }
    static func ec4(_ s: Int, _ g: Int, _ e: Int) -> String { "ec4:\(s):\(g):\(e)" }
    static func ec4Push(_ s: Int, _ g: Int, _ e: Int) -> String { "ec4p:\(s):\(g):\(e)" }

    func note(_ key: String) -> String? {
        guard let n = notes[key]?.trimmingCharacters(in: .whitespacesAndNewlines), !n.isEmpty else { return nil }
        return n
    }

    func binding(_ key: String) -> Binding<String> {
        Binding(get: { self.notes[key] ?? "" },
                set: { v in
                    if v.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty { self.notes[key] = nil } else { self.notes[key] = v }
                })
    }

    /// One line, for comments and small labels.
    static func oneLine(_ s: String) -> String {
        s.replacingOccurrences(of: "\n", with: " ").replacingOccurrences(of: "  ", with: " ")
    }

    private static var url: URL {
        let dir = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("DualSense Mapper", isDirectory: true)
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir.appendingPathComponent("notes.json")
    }

    private var loading = false
    private func load() {
        loading = true
        defer { loading = false }
        if let d = try? Data(contentsOf: NotesStore.url), let n = try? JSONDecoder().decode([String: String].self, from: d) {
            notes = n
        }
    }

    private func save() {
        guard !loading else { return }
        let enc = JSONEncoder(); enc.outputFormatting = [.prettyPrinted, .sortedKeys]
        if let d = try? enc.encode(notes) { try? d.write(to: NotesStore.url, options: .atomic) }
    }
}

/// Note editor used in all inspectors.
struct NoteField: View {
    @Binding var text: String
    var title = "Note"
    var prompt = "What is this for? e.g. “grain duration on the ER-301”"

    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            Label(title, systemImage: "pencil.line")
                .font(Theme.font(10, .heavy, .mono))
                .foregroundStyle(Theme.note)
                .tracking(1)
            TextField("", text: $text, prompt: Text(prompt).foregroundStyle(Theme.dim.opacity(0.7)), axis: .vertical)
                .lineLimit(1...5)
                .font(Theme.font(12, .regular, .ui))
                .textFieldStyle(.plain)
                .padding(.horizontal, 8).padding(.vertical, 6)
                .background(RoundedRectangle(cornerRadius: 6).fill(Theme.bg.opacity(0.7)))
                .overlay(RoundedRectangle(cornerRadius: 6).stroke(Theme.note.opacity(text.isEmpty ? 0.15 : 0.45), lineWidth: 1))
        }
    }
}

/// Small note line on the drawings.
struct NoteBadge: View {
    let text: String
    var size: CGFloat = 9.5
    var lines = 1
    var body: some View {
        Text("✎ " + NotesStore.oneLine(text))
            .font(Theme.font(size, .medium, .ui).italic())
            .foregroundStyle(Theme.note)
            .lineLimit(lines)
            .truncationMode(.tail)
            .help(text)
    }
}

