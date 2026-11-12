import SwiftUI
import AppKit

@main
struct DualSenseMapperApp: App {
    @NSApplicationDelegateAdaptor(AppDelegate.self) private var delegate
    @State private var mapping = Mapping()
    @State private var bank = Faderbank()
    @State private var pedals = PedalBoard()
    @State private var ec4 = EC4Board()
    @State private var notes = NotesStore()

    var body: some Scene {
        WindowGroup("Kontrolleur") {
            ContentView()
                .environment(mapping)
                .environment(bank)
                .environment(pedals)
                .environment(ec4)
                .environment(notes)
                .onAppear { AppDelegate.mapping = mapping }
                .frame(minWidth: 960, minHeight: 640)
        }
        .windowStyle(.hiddenTitleBar)
        .defaultSize(width: 1400, height: 840)
    }
}

final class AppDelegate: NSObject, NSApplicationDelegate {
    @MainActor static var mapping: Mapping?
    func applicationDidFinishLaunching(_ notification: Notification) {
        NSApp.setActivationPolicy(.regular)
        NSApp.activate(ignoringOtherApps: true)
        // DSM_SNAPSHOT=/path/shot.png renders the window to a PNG after a few seconds and quits (for checking the UI).
        if let path = ProcessInfo.processInfo.environment["DSM_SNAPSHOT"] {
            let wait = Double(ProcessInfo.processInfo.environment["DSM_WAIT"] ?? "") ?? 5
            DispatchQueue.main.asyncAfter(deadline: .now() + wait) {
                if let out = ProcessInfo.processInfo.environment["DSM_CONFIG_TEST"], let m = AppDelegate.mapping {
                    // round trip: mapping → configuration file → fresh mapping; the Pi files must be identical
                    var c = ConfigFile(saved: Date()); c.mapping = m.snapshot
                    let url = URL(fileURLWithPath: out + ".dsmconfig")
                    var result = "write failed"
                    if (try? ConfigLibrary.write(c, to: url)) != nil, let back = try? ConfigLibrary.read(url), let snap = back.mapping {
                        let m2 = Mapping(); m2.ccNames = m.ccNames; m2.nrpnNames = m.nrpnNames; m2.controlsOnPi = m.controlsOnPi
                        m2.restore(snap)
                        result = "routing identical: \(m2.pythonSource() == m.pythonSource())\ncontrols identical: \(m2.controlsSource() == m.controlsSource())\nexternals: \(m.externals.count) funcs: \(m.funcs.count) chords: \(m.chords.count)\n"
                    }
                    try? result.write(toFile: out, atomically: true, encoding: .utf8)
                }
                if let dump = ProcessInfo.processInfo.environment["DSM_DUMP"], let m = AppDelegate.mapping {
                    try? m.pythonSource().write(toFile: dump, atomically: true, encoding: .utf8)
                    try? m.controlsSource().write(toFile: dump + ".controls.py", atomically: true, encoding: .utf8)
                }
                if let info = ProcessInfo.processInfo.environment["DSM_WININFO"], let w = NSApp.windows.first(where: { $0.isVisible }) {
                    let t = "frame \(w.frame.size) min \(w.contentMinSize) max \(w.contentMaxSize) minSize \(w.minSize) maxSize \(w.maxSize) resizable \(w.styleMask.contains(.resizable)) fitting \(w.contentView?.fittingSize ?? .zero)\n"
                    try? t.write(toFile: info, atomically: true, encoding: .utf8)
                }
                if let w = NSApp.windows.first(where: { $0.isVisible }), let v = w.contentView {
                    let dims = (ProcessInfo.processInfo.environment["DSM_SIZE"] ?? "1400x840").split(separator: "x").compactMap { Double($0) }
                    w.setContentSize(NSSize(width: dims.first ?? 1400, height: dims.last ?? 840))
                    v.layoutSubtreeIfNeeded()
                    if let rep = v.bitmapImageRepForCachingDisplay(in: v.bounds) {
                        v.cacheDisplay(in: v.bounds, to: rep)
                        try? rep.representation(using: .png, properties: [:])?.write(to: URL(fileURLWithPath: path))
                    }
                }
                NSApp.terminate(nil)
            }
        }
    }
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
}

struct Status: Equatable {
    enum Kind { case info, ok, warn, error }
    var kind: Kind
    var text: String
    var time = Date()
}

@MainActor
struct ContentView: View {
    @Environment(Mapping.self) private var mapping
    @Environment(Faderbank.self) private var bank
    @Environment(PedalBoard.self) private var pedals
    @Environment(EC4Board.self) private var ec4
    @Environment(NotesStore.self) private var notes
    @AppStorage("ec4OnPi") private var ec4OnPi = true
    @State private var ec4Setup = 0
    @State private var ec4Group = 0
    @State private var ec4Enc = 0
    @State private var syncedEC4: String? = nil
    @State private var confirmEC4Send = false
    @State private var pendingConfig: PendingConfig? = nil
    @State private var er301Tab: ER301Tab = .ports
    @State private var selectedPort: String? = nil
    @AppStorage("pedalsOnPi") private var pedalsOnPi = true
    @State private var selectedPedal = 0
    @State private var syncedPedals: String? = nil
    @AppStorage("deviceMode") private var deviceMode = "dualsense"
    @AppStorage("fader16OnPi") private var fader16OnPi = true
    @AppStorage("python") private var python = "/usr/bin/python3"
    @State private var selectedFader = 0
    @State private var syncedBank: String? = nil
    @AppStorage("host") private var host = "ahab@ahab.local"
    @AppStorage("identity") private var identity = "~/.ssh/id_ed25519_pi"
    @AppStorage("repo") private var repo = "~/dualsense-eurorack-midi"

    @State private var selected: GroupID = .cross
    @State private var tab: PanelTab = .control
    @State private var busy = false
    @State private var status = Status(kind: .info, text: "Loading …")
    @State private var synced: String? = nil
    @State private var showSettings = false
    @State private var showAppearance = false
    @State private var winWidth: CGFloat = 1400
    private var compact: Bool { winWidth < 1290 }

    private var client: PiClient { PiClient(host: host, identity: identity, repo: repo) }
    private var dirty: Bool {
        (synced != nil && synced != currentSources) || (syncedBank != nil && syncedBank != bank.json)
            || (syncedPedals != nil && syncedPedals != pedals.json)
            || (syncedEC4 != nil && syncedEC4 != ec4.fingerprint)
    }
    private var ec4Client: EC4Client { EC4Client(onPi: ec4OnPi, pi: client, python: python, repo: repo) }
    private var pedalClient: PedalClient { PedalClient(onPi: pedalsOnPi, pi: client, python: python, repo: repo) }
    private var faderClient: FaderClient { FaderClient(onPi: fader16OnPi, pi: client, python: python, repo: repo) }
    private var currentSources: String { mapping.pythonSource() + "\n#----\n" + mapping.controlsSource() }

    var body: some View {
        VStack(spacing: 0) {
            topBar
            Divider().overlay(Theme.line)
            HStack(spacing: 0) {
                if deviceMode == "er301" {
                    ER301Canvas(selectedPort: $selectedPort)
                        .padding(.horizontal, 18).padding(.vertical, 14)
                    Divider().overlay(Theme.line)
                    ER301Panel(tab: $er301Tab, selectedPort: selectedPort)
                } else if deviceMode == "ec4" {
                    EC4Canvas(setup: $ec4Setup, group: $ec4Group, selected: $ec4Enc)
                        .padding(.horizontal, 18).padding(.vertical, 14)
                    Divider().overlay(Theme.line)
                    EC4Panel(setup: ec4Setup, group: ec4Group, selected: $ec4Enc)
                } else if deviceMode == "pedals" {
                    PedalCanvas(selected: $selectedPedal)
                        .padding(.horizontal, 18).padding(.vertical, 14)
                    Divider().overlay(Theme.line)
                    PedalPanel(selected: $selectedPedal)
                } else if deviceMode == "faderbank" {
                    FaderbankCanvas(selected: $selectedFader)
                        .padding(.horizontal, 18).padding(.vertical, 14)
                    Divider().overlay(Theme.line)
                    FaderPanel(selected: $selectedFader)
                } else {
                    ControllerCanvas(selected: $selected)
                        .padding(.horizontal, 18).padding(.vertical, 14)
                        .onChange(of: selected) { tab = .control }
                    Divider().overlay(Theme.line)
                    SidePanel(selected: $selected, tab: $tab)
                }
            }
            Divider().overlay(Theme.line)
            statusBar
        }
        .background(Theme.bg)
        .background(GeometryReader { g in
            Color.clear
                .onAppear { winWidth = g.size.width }
                .onChange(of: g.size.width) { winWidth = g.size.width }
        })
        .preferredColorScheme(Appearance.shared.colorScheme)
        .tint(Theme.cv)
        .task {
            if deviceMode == "faderbank" {
                await load16n()              // the 16n first: the Pi may be unreachable and take a while
                await load(quiet: true)
            } else if deviceMode == "pedals" {
                await loadPedals()
                await load(quiet: true)
            } else if deviceMode == "ec4" {
                loadEC4Cache()
                await load(quiet: true)
            } else {
                await load()
            }
        }
        .onChange(of: deviceMode) {
            if deviceMode == "faderbank" && !bank.loaded { Task { await load16n() } }
            if deviceMode == "pedals" && !pedals.loaded { Task { await loadPedals() } }
            if deviceMode == "ec4" && !ec4.loaded { loadEC4Cache() }
        }
        .onChange(of: pedalsOnPi) {
            if deviceMode == "pedals" { Task { await loadPedals() } }
        }
        .onChange(of: fader16OnPi) {
            if deviceMode == "faderbank" { Task { await load16n() } }
        }
        .onAppear {      // DSM_SELECT / DSM_TAB: preselect for UI snapshots
            let env = ProcessInfo.processInfo.environment
            if let g = env["DSM_SELECT"].flatMap(GroupID.init(rawValue:)) { selected = g }
            if let t = env["DSM_TAB"].flatMap(PanelTab.init(rawValue:)) { DispatchQueue.main.async { tab = t } }
        }
    }

    // MARK: bars

    private var topBar: some View {
        HStack(spacing: 10) {
            VStack(alignment: .leading, spacing: 1) {
                Text("Kontrolleur").font(Theme.font(17, .bold, .title)).foregroundStyle(Theme.ink)
                if !compact {
                    Text("→ i2c2midi → ER-301").font(Theme.font(10, .semibold, .mono))
                        .foregroundStyle(Theme.dim).tracking(1)
                }
            }
            .lineLimit(1)
            .fixedSize()
            .padding(.leading, 70)     // traffic lights
            Picker("", selection: $deviceMode) {
                Text("DualSense").tag("dualsense")
                Text("16n").tag("faderbank")
                Text("Pedals").tag("pedals")
                Text("EC4").tag("ec4")
                Text("ER-301").tag("er301")
            }
            .pickerStyle(.segmented).labelsHidden().frame(width: compact ? 310 : 360)
            .padding(.leading, compact ? 6 : 18)
            if ["faderbank", "pedals", "ec4"].contains(deviceMode) {
                Picker("", selection: deviceMode == "pedals" ? $pedalsOnPi : (deviceMode == "ec4" ? $ec4OnPi : $fader16OnPi)) {
                    Label("Pi", systemImage: "cpu").tag(true)
                    Label("Mac", systemImage: "laptopcomputer").tag(false)
                }
                .pickerStyle(.segmented).labelsHidden().frame(width: compact ? 84 : 132)
                .labelStyle(compact ? AnyLabelStyle(.iconOnly) : AnyLabelStyle(.titleAndIcon))
                .disabled(busy)
                .help("Where this device is plugged in: the Raspberry Pi (default) or this Mac")
            }
            Spacer(minLength: 4)
            if dirty {
                Label("Unsaved changes", systemImage: "circle.fill")
                    .labelStyle(compact ? AnyLabelStyle(.iconOnly) : AnyLabelStyle(.titleAndIcon))
                    .help("Unsaved changes: Upload sends them")
                    .font(Theme.font(11, .semibold, .ui))
                    .foregroundStyle(Theme.trig)
                    .imageScale(.small)
            }
            configMenu
            Button { showAppearance.toggle() } label: { Image(systemName: "paintbrush") }
                .buttonStyle(.bordered)
                .help("Appearance: colours, fonts, text size")
                .popover(isPresented: $showAppearance, arrowEdge: .bottom) { AppearancePanel() }
            Button { showSettings.toggle() } label: {
                Image(systemName: "network")
            }
            .help("Raspberry Pi: \(host) (click to change)")
            .buttonStyle(.bordered)
            .popover(isPresented: $showSettings, arrowEdge: .bottom) { settings }

            Button { Task {
                if deviceMode == "faderbank" { await load16n(); await load(quiet: true) }
                else if deviceMode == "pedals" { await loadPedals(); await load(quiet: true) }
                else if deviceMode == "ec4" { await receiveEC4() }
                else { await load() }
            } } label: {
                Label(deviceMode == "ec4" ? "Receive" : "Load", systemImage: "arrow.down.circle").fixedSize()
                    .labelStyle(compact ? AnyLabelStyle(.iconOnly) : AnyLabelStyle(.titleAndIcon))
            }
            .help(deviceMode == "ec4" ? "Receive the setups from the EC4 (⌘R)" : "Load from the device (⌘R)")
                .buttonStyle(.bordered)
                .keyboardShortcut("r", modifiers: .command)
                .disabled(busy)

            Button { Task {
                if deviceMode == "faderbank" { await upload16n() }
                else if deviceMode == "pedals" { await uploadPedals() }
                else if deviceMode == "ec4" { confirmEC4Send = true }
                else { await upload() }
            } } label: {
                Label(deviceMode == "ec4" ? "Send" : "Upload", systemImage: "arrow.up.circle.fill").font(Theme.font(13, .semibold, .ui)).fixedSize()
                    .labelStyle(compact ? AnyLabelStyle(.iconOnly) : AnyLabelStyle(.titleAndIcon))
            }
            .help(deviceMode == "ec4" ? "Send the setups to the EC4 (⌘S)" : "Upload to the device (⌘S)")
            .buttonStyle(.borderedProminent)
            .tint(Theme.cv.opacity(0.85))
            .keyboardShortcut("s", modifiers: .command)
            .disabled(busy)
            .alert("Send all setups to the EC4?", isPresented: $confirmEC4Send) {
                Button("Send") { Task { await sendEC4() } }
                Button("Cancel", role: .cancel) {}
            } message: {
                Text("First on the EC4: press Func, choose Setup, then Receive (the display shows “Work in progress”). Then press Send. This replaces all 16 setups on the device; the last received data is kept on this Mac as a backup.")
            }
        }
        .padding(.horizontal, 16).padding(.vertical, 10)
        .background(Theme.panel)
    }

    // MARK: configurations (saved on this Mac)

    private var configMenu: some View {
        Menu {
            Button("Save Configuration…") { saveConfig() }
            Button("Open Configuration…") { ConfigLibrary.askOpen { openConfig($0) } }
            let recent = ConfigLibrary.recent()
            if !recent.isEmpty {
                Divider()
                Section("Recent") {
                    ForEach(recent, id: \.self) { u in
                        Button(u.deletingPathExtension().lastPathComponent) { openConfig(u) }
                    }
                }
            }
            Divider()
            Button("Show Configurations in Finder") { NSWorkspace.shared.open(ConfigLibrary.folder) }
        } label: {
            Label("Configs", systemImage: "folder")
                .labelStyle(compact ? AnyLabelStyle(.iconOnly) : AnyLabelStyle(.titleAndIcon))
        }
        .menuStyle(.borderlessButton)
        .fixedSize()
        .background {
            // keyboard shortcuts for the menu items
            Button("") { saveConfig() }.keyboardShortcut("s", modifiers: [.command, .shift]).opacity(0).frame(width: 0, height: 0)
            Button("") { ConfigLibrary.askOpen { openConfig($0) } }.keyboardShortcut("o", modifiers: .command).opacity(0).frame(width: 0, height: 0)
        }
        .help("Save the whole setup (DualSense, 16n, pedals, EC4, notes) on this Mac, or load one (⌘⇧S / ⌘O)")
        .sheet(item: $pendingConfig) { pc in
            ConfigLoadSheet(pending: pc) { parts in applyConfig(pc, parts) }
        }
    }

    private func saveConfig() {
        var c = ConfigFile(saved: Date())
        c.mapping = mapping.snapshot
        if bank.loaded { c.faderbank = .init(faders: bank.faders, options: bank.options) }
        if pedals.loaded { c.pedals = pedals.inputs }
        if ec4.loaded { c.ec4 = .init(mem: ec4.mem, version: ec4.version, setups: ec4.setups) }
        c.notes = notes.notes
        let f = DateFormatter(); f.dateFormat = "yyyy-MM-dd HHmm"
        ConfigLibrary.askSave(suggested: "Setup \(f.string(from: Date()))") { url in
            do {
                try ConfigLibrary.write(c, to: url)
                let parts = ConfigFile.Part.allCases.filter { c.has($0) }.map(\.rawValue)
                status = Status(kind: .ok, text: "Saved “\(url.deletingPathExtension().lastPathComponent)”: \(parts.joined(separator: ", ")).")
            } catch {
                status = Status(kind: .error, text: "Could not save the configuration: \(error.localizedDescription)")
            }
        }
    }

    private func openConfig(_ url: URL) {
        do {
            pendingConfig = PendingConfig(url: url, file: try ConfigLibrary.read(url))
        } catch {
            status = Status(kind: .error, text: "Could not open “\(url.lastPathComponent)”: \(error.localizedDescription)")
        }
    }

    private func applyConfig(_ pc: PendingConfig, _ parts: Set<ConfigFile.Part>) {
        let c = pc.file
        var done: [String] = []
        if parts.contains(.dualsense), let m = c.mapping { mapping.restore(m); done.append("DualSense") }
        if parts.contains(.faderbank), let f = c.faderbank, f.faders.count == 16 {
            bank.faders = f.faders; bank.options = f.options; bank.loaded = true; done.append("16n")
        }
        if parts.contains(.pedals), let p = c.pedals, p.count >= 4 { pedals.apply(p); done.append("pedals") }
        if parts.contains(.ec4), let e = c.ec4, e.setups.count == 16 {
            ec4.apply(EC4DTO(ok: true, error: nil, version: e.version, port: nil, mem: e.mem, setups: e.setups))
            done.append("EC4")
        }
        if parts.contains(.notes), let n = c.notes { notes.notes = n; done.append("notes") }
        let name = pc.url.deletingPathExtension().lastPathComponent
        status = Status(kind: .ok, text: "Loaded \(done.joined(separator: ", ")) from “\(name)”. Send it to the devices with “Upload” on each tab.")
    }

    private var settings: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Raspberry Pi").font(.headline)
            LabeledContent("SSH host") { TextField("user@host", text: $host).frame(width: 220) }
            LabeledContent("SSH key") { TextField("~/.ssh/…", text: $identity).frame(width: 220) }
            LabeledContent("Repo on the Pi") { TextField("~/dualsense-eurorack-midi", text: $repo).frame(width: 220) }
            Text("Used by every tab. Leave the key empty to use your ~/.ssh/config.").font(.caption).foregroundStyle(.secondary)
            Divider()
            Text("Where each device is plugged in").font(.headline)
            Text("One setting per device: the same as the Pi | Mac switch on that device's tab. The DualSense and the ER-301 routing always go through the Pi.")
                .font(.caption).foregroundStyle(.secondary).fixedSize(horizontal: false, vertical: true)
            plugRow("16n faderbank", $fader16OnPi)
            plugRow("MPC-20 pedals", $pedalsOnPi)
            plugRow("Faderfox EC4", $ec4OnPi)
            Divider()
            Text("This Mac").font(.headline)
            LabeledContent("Python") { TextField("/usr/bin/python3", text: $python).frame(width: 220) }
            Text("For devices plugged into this Mac. 16n and EC4 need python-rtmidi; the MPC-20 needs nothing extra.")
                .font(.caption).foregroundStyle(.secondary).fixedSize(horizontal: false, vertical: true)
        }
        .padding(16)
        .frame(width: 380)
    }

    private func plugRow(_ title: String, _ onPi: Binding<Bool>) -> some View {
        LabeledContent(title) {
            Picker("", selection: onPi) {
                Text("Pi").tag(true)
                Text("this Mac").tag(false)
            }
            .pickerStyle(.segmented).labelsHidden().frame(width: 150)
        }
    }

    private var statusBar: some View {
        HStack(spacing: 8) {
            if busy { ProgressView().controlSize(.small) }
            else {
                Image(systemName: icon).foregroundStyle(statusColor)
            }
            Text(status.text).font(Theme.font(12, .regular, .ui)).foregroundStyle(Theme.ink.opacity(0.9)).lineLimit(2)
            Spacer()
            Text(status.time, style: .time).font(Theme.font(11, .regular, .mono)).foregroundStyle(Theme.dim)
        }
        .padding(.horizontal, 16).padding(.vertical, 8)
        .background(Theme.panel)
    }

    private var icon: String {
        switch status.kind {
        case .info: "info.circle"
        case .ok: "checkmark.circle.fill"
        case .warn: "exclamationmark.triangle.fill"
        case .error: "xmark.octagon.fill"
        }
    }
    private var statusColor: Color {
        switch status.kind {
        case .info: Theme.dim
        case .ok: Theme.special
        case .warn: Theme.trig
        case .error: Color(red: 1, green: 0.38, blue: 0.38)
        }
    }

    // MARK: actions

    /// quiet: keep the current status line unless something goes wrong (used behind the 16n load).
    private func load(quiet: Bool = false) async {
        let before = status
        busy = true
        defer {
            busy = false
            if quiet {
                if status.kind == .error && before.kind != .error {
                    status = Status(kind: .warn, text: before.text + " The Pi isn't reachable, so fader → ER-301 routes can't be loaded or uploaded right now.")
                } else {
                    status = before
                }
            }
        }
        do {
            let e: ExportDTO
            if let f = ProcessInfo.processInfo.environment["DSM_IMPORT"] {      // offline test: Pi JSON from a file
                e = try JSONDecoder().decode(ExportDTO.self, from: Data(contentsOf: URL(fileURLWithPath: f)))
            } else {
                e = try await client.export()
            }
            if e.cc_names == nil {
                status = Status(kind: .error, text: "The Pi answered, but: \(e.error ?? "no data")")
                return
            }
            mapping.apply(e)
            synced = currentSources
            if let ce = e.controls_error {
                status = Status(kind: .warn, text: "Loaded, but controls.py on the Pi has a problem (defaults shown): \(ce)")
            } else if !mapping.controlsOnPi {
                status = Status(kind: .warn, text: "Loaded the routing. The Pi doesn't have the button functions installed yet: only the routing will be uploaded.")
            } else if e.ok {
                status = Status(kind: .ok, text: "Loaded routing and button functions from \(host).")
            } else {
                status = Status(kind: .warn, text: "Loaded, but the file on the Pi has a problem: \(e.error ?? "")")
            }
        } catch {
            status = Status(kind: .error, text: "Could not load from \(host): \(error.localizedDescription)")
        }
    }

    private func load16n() async {
        busy = true
        defer { busy = false }
        do {
            let d = try await faderClient.export()
            guard d.ok else {
                status = Status(kind: .error, text: "16n (\(fader16OnPi ? host : "this Mac")): \(d.error ?? "no answer")")
                return
            }
            bank.apply(d)
            syncedBank = bank.json
            let fw = "\(bank.device), firmware \(bank.firmware)\(bank.curvesSupported ? " + curves" : " (no curves)")"
            status = Status(kind: bank.curvesSupported ? .ok : .warn, text: "Loaded the 16n (\(fw)) via \(fader16OnPi ? host : "this Mac").")
        } catch {
            status = Status(kind: .error, text: "Could not read the 16n via \(fader16OnPi ? host : "this Mac"): \(error.localizedDescription)")
        }
    }

    /// Notes of routed controls, for the comments in er301_routes.py.
    private func routeNotes() -> [String: String] {
        var r: [String: String] = [:]
        for k in notes.notes.keys where k.hasPrefix("ds:") { if let n = notes.note(k) { r[k] = n } }
        func ext(_ ch: Int, _ cc: Int, _ key: String) {
            if r["ext:\(ch):\(cc)"] == nil, let n = notes.note(key) { r["ext:\(ch):\(cc)"] = n }
        }
        for (i, f) in bank.faders.enumerated() { ext(f.usb_channel, f.usb_cc, NotesStore.fader(i)) }
        for inp in pedals.inputs {
            for s in inp.slots where s.on && s.kind == "cc" {
                ext(PedalBoard.hubChannel(s.channel, mapping: mapping), s.number, NotesStore.pedal(inp.input, s.slot))
            }
        }
        for (si, su) in ec4.setups.enumerated() {
            for (gi, g) in su.groups.enumerated() {
                for (ei, e) in g.encoders.enumerated() where e.type == "cc" { ext(e.channel, e.number, NotesStore.ec4(si, gi, ei)) }
            }
        }
        return r
    }

    private func loadEC4Cache() {
        if ec4.loadCache() {
            syncedEC4 = ec4.fingerprint
            status = Status(kind: .info, text: "Showing the EC4 setups received \(ec4.receivedAt.map { $0.formatted(date: .abbreviated, time: .shortened) } ?? "earlier"). Press “Receive” to fetch them again.")
        } else {
            status = Status(kind: .info, text: "Press “Receive”, then on the EC4: Func → Setup → Send → “Send all setups” (hold the knob).")
        }
    }

    private func receiveEC4() async {
        busy = true
        defer { busy = false }
        let via = ec4OnPi ? host : "this Mac"
        status = Status(kind: .info, text: "Waiting for the EC4 (via \(via)): on the device press Func → Setup → Send → “Send all setups” and hold the knob …")
        do {
            let d = try await ec4Client.receive(seconds: 120)
            guard d.ok else { status = Status(kind: .error, text: "EC4: \(d.error ?? "nothing received")"); return }
            ec4.apply(d)
            ec4.receivedAt = Date()
            ec4.saveCache()
            syncedEC4 = ec4.fingerprint
            let used = ec4.setups.filter { !$0.name.hasPrefix("SE") && !$0.name.hasPrefix("--") }.count
            status = Status(kind: .ok, text: "Received all 16 setups from the EC4 via \(via) (\(used) named). Kept on this Mac as a backup.")
        } catch {
            status = Status(kind: .error, text: "Could not receive from the EC4 via \(via): \(error.localizedDescription)")
        }
    }

    private func sendEC4() async {
        if let p = ec4.problems.first { status = Status(kind: .error, text: p); return }
        if !ec4.loaded { status = Status(kind: .error, text: "Receive the setups from the EC4 first."); return }
        let via = ec4OnPi ? host : "this Mac"
        busy = true
        status = Status(kind: .info, text: "Sending all setups to the EC4 via \(via) … (about a minute; the EC4 shows the progress)")
        do {
            let r = try await ec4Client.send(ec4.json)
            busy = false
            guard r.ok else { status = Status(kind: .error, text: "EC4 not written: \(r.error ?? "unknown error")"); return }
            ec4.saveCache()
            syncedEC4 = ec4.fingerprint
            var text = "Sent all setups to the EC4 (\(r.changed_bytes ?? 0) bytes changed). Check its display for “done”."
            if synced != nil && synced != currentSources {
                await upload()                                   // encoder → ER-301 routes changed too
                text += " " + status.text
            }
            status = Status(kind: .ok, text: text)
        } catch {
            busy = false
            status = Status(kind: .error, text: "Sending to the EC4 via \(via) failed: \(error.localizedDescription)")
        }
    }

    private func loadPedals() async {
        busy = true
        defer { busy = false }
        let via = pedalsOnPi ? host : "this Mac"
        do {
            let d = try await pedalClient.export()
            guard d.ok else {
                status = Status(kind: .error, text: "MPC-20 (\(via)): \(d.error ?? "no answer")")
                return
            }
            pedals.apply(d)
            syncedPedals = pedals.json
            let used = pedals.inputs.prefix(4).filter { $0.type_id != 0 }.count
            status = Status(kind: .ok, text: "Loaded the \(pedals.device) (firmware \(pedals.firmware)) via \(via): \(used) of 4 jacks in use.")
        } catch {
            status = Status(kind: .error, text: "Could not read the MPC-20 via \(via): \(error.localizedDescription)")
        }
    }

    private func uploadPedals() async {
        if let p = pedals.problems.first { status = Status(kind: .error, text: p); return }
        if !pedals.loaded { status = Status(kind: .error, text: "Load the MPC-20 first."); return }
        let via = pedalsOnPi ? host : "this Mac"
        busy = true
        do {
            let r = try await pedalClient.install(pedals.json)
            busy = false
            guard r.ok else {
                let why = r.error ?? ("read-back differs: " + (r.mismatch ?? []).joined(separator: ", "))
                status = Status(kind: .error, text: "MPC-20 not written correctly: \(why)")
                return
            }
            if let l = r.inputs { pedals.apply(l) }
            syncedPedals = pedals.json
            var text = "MPC-20 written and verified (\(r.written ?? 0) message\((r.written ?? 0) == 1 ? "" : "s") changed)."
            if let s = r.skipped, !s.isEmpty { text += " Skipped (nothing plugged in): \(s.joined(separator: ", "))." }
            if synced != nil && synced != currentSources {
                await upload()                                   // pedal → ER-301 routes changed too
                status = Status(kind: status.kind, text: text + " " + status.text)
            } else {
                status = Status(kind: .ok, text: text)
            }
        } catch {
            busy = false
            status = Status(kind: .error, text: "MPC-20 upload via \(via) failed: \(error.localizedDescription)")
        }
    }

    private func upload16n() async {
        if let p = bank.problems.first { status = Status(kind: .error, text: p); return }
        if !bank.loaded { status = Status(kind: .error, text: "Load the 16n first, so its current settings aren't overwritten with defaults."); return }
        busy = true
        do {
            let r = try await faderClient.install(bank.json)
            busy = false
            guard r.ok else {
                let why = r.error ?? ("read-back differs: " + (r.mismatch ?? []).joined(separator: ", "))
                status = Status(kind: .error, text: "16n not written correctly: \(why)")
                return
            }
            syncedBank = bank.json
            var text = "16n written and verified."
            if let s = r.skipped, !s.isEmpty { text += " Not supported by its firmware: \(s.joined(separator: ", "))." }
            if synced != nil && synced != currentSources {
                await upload()                                   // fader → ER-301 routes changed too
                status = Status(kind: status.kind, text: text + " " + status.text)
            } else {
                status = Status(kind: .ok, text: text)
            }
        } catch {
            busy = false
            status = Status(kind: .error, text: "16n upload via \(fader16OnPi ? host : "this Mac") failed: \(error.localizedDescription)")
        }
    }

    private func upload() async {
        if let p = mapping.problems.first {
            status = Status(kind: .error, text: p)
            return
        }
        busy = true
        defer { busy = false }
        let source = mapping.pythonSource(notes: routeNotes())
        let controls = mapping.controlsSource()
        let sentAll = currentSources
        do {
            let r = try await client.install(source)
            guard r.ok else {
                status = Status(kind: .error, text: "The Pi rejected the routing: \(r.error ?? "unknown error")")
                return
            }
            var parts: [String] = []
            var kind = Status.Kind.ok
            if r.hub != nil { parts.append("Routing sent to the i2c2midi.") }
            else { parts.append("Routing saved, but the hub did not confirm (is the XIAO-TT connected?)."); kind = .warn }
            if mapping.controlsOnPi {
                let c = try await client.installControls(controls)
                if !c.ok {
                    status = Status(kind: .error, text: parts.joined(separator: " ") + " Button functions rejected: \(c.error ?? "unknown error")")
                    return
                }
                if let line = c.hub, line.contains("NOT applied") {
                    parts.append("The DualSense script refused the button functions: \(line)"); kind = .error
                } else if c.hub != nil { parts.append("Button functions active.") }
                else { parts.append("Button functions saved; the DualSense script did not confirm (is it running?)."); if kind == .ok { kind = .warn } }
            } else {
                parts.append("Button functions not uploaded (not installed on the Pi yet).")
                if kind == .ok { kind = .warn }
            }
            synced = sentAll
            status = Status(kind: kind, text: parts.joined(separator: " "))
        } catch {
            status = Status(kind: .error, text: "Upload to \(host) failed: \(error.localizedDescription)")
        }
    }
}


/// Switch label styles at runtime (compact toolbar).
struct AnyLabelStyle: LabelStyle {
    private let make: (Configuration) -> AnyView
    init<S: LabelStyle>(_ style: S) { make = { AnyView(style.makeBody(configuration: $0)) } }
    func makeBody(configuration: Configuration) -> some View { make(configuration) }
}
