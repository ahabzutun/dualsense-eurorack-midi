import SwiftUI
import AppKit

/// Mutable state the scroll-wheel monitor reads (it lives outside SwiftUI updates).
final class StageBox {
    var size: CGSize = .zero
    var hover: CGPoint? = nil
    var monitor: Any? = nil
}

/// Shows a drawing made for a fixed size, fitted and centred in the available space.
/// Zoom: pinch (trackpad), mouse / trackball scroll wheel (towards the pointer), ⌥ or ⌘ + scroll,
/// the buttons in the corner or ⌘+ / ⌘− / ⌘0. Pan: drag, or two-finger scroll on the trackpad.
struct ZoomStage<Content: View>: View {
    let size: CGSize
    /// true: the content is a flow layout with its own natural size (it grows with the text size
    /// and font); the stage becomes at least that big, centred, on the blueprint grid.
    var grows = false
    @ViewBuilder var content: () -> Content

    @State private var natural: CGSize = .zero
    /// The drawing's size in drawing units: `size`, or bigger when the content needs more room.
    private var stage: CGSize {
        guard grows else { return size }
        return CGSize(width: max(size.width, natural.width + 60), height: max(size.height, natural.height + 60))
    }

    // DSM_ZOOM / DSM_PAN ("x,y") preset the view for UI snapshots
    @State private var zoom: CGFloat = CGFloat(Double(ProcessInfo.processInfo.environment["DSM_ZOOM"] ?? "") ?? 1)
    @State private var offset: CGSize = {
        let p = (ProcessInfo.processInfo.environment["DSM_PAN"] ?? "").split(separator: ",").compactMap { Double($0) }
        return p.count == 2 ? CGSize(width: p[0], height: p[1]) : .zero
    }()
    @State private var pinchBase: (zoom: CGFloat, offset: CGSize)? = nil
    @State private var dragBase: CGSize? = nil
    @State private var box = StageBox()
    private let maxZoom: CGFloat = 6

    var body: some View {
        GeometryReader { geo in
            let s = stage
            let fit = max(0.05, min(geo.size.width / s.width, geo.size.height / s.height))
            Group {
                if grows {
                    ZStack {
                        BlueprintGrid()
                        content()
                            .fixedSize()
                            .background(GeometryReader { g in Color.clear.preference(key: NaturalSizeKey.self, value: g.size) })
                    }
                    .onPreferenceChange(NaturalSizeKey.self) { natural = $0 }
                } else {
                    content()
                }
            }
                .id(Appearance.shared.revision)          // redraw the drawing when colours / fonts change
                .frame(width: s.width, height: s.height)
                .scaleEffect(fit * zoom)                 // around the centre, so it stays centred
                .offset(offset)
                .frame(width: geo.size.width, height: geo.size.height)
                .clipped()
                .contentShape(Rectangle())
                .gesture(magnify(geo, fit))
                .simultaneousGesture(pan(geo, fit))
                .overlay(alignment: .bottomTrailing) { controls(geo, fit).padding(10) }
                .onContinuousHover(coordinateSpace: .local) { phase in
                    switch phase {
                    case .active(let p): box.hover = p
                    case .ended: box.hover = nil
                    }
                }
                .onAppear { box.size = geo.size; installScroll() }
                .onDisappear { removeScroll() }
                .onChange(of: geo.size) { box.size = geo.size; offset = clamp(offset, geo.size, zoom) }
        }
    }

    // MARK: scroll wheel / trackpad scrolling

    private func installScroll() {
        guard box.monitor == nil else { return }
        let box = box
        box.monitor = NSEvent.addLocalMonitorForEvents(matching: .scrollWheel) { ev in
            guard let p = box.hover, box.size.width > 0 else { return ev }
            let zoomKey = ev.modifierFlags.contains(.option) || ev.modifierFlags.contains(.command)
            if ev.hasPreciseScrollingDeltas && !zoomKey {
                // trackpad / smooth scrolling: pan while zoomed in
                guard zoom > 1.001 else { return ev }
                offset = clamp(CGSize(width: offset.width + ev.scrollingDeltaX, height: offset.height + ev.scrollingDeltaY), box.size, zoom)
                return nil
            }
            // mouse / trackball wheel (or ⌥/⌘ + scroll): zoom towards the pointer
            let d = ev.hasPreciseScrollingDeltas ? ev.scrollingDeltaY / 40 : ev.scrollingDeltaY
            guard d != 0 else { return nil }
            let z = min(maxZoom, max(1, zoom * pow(1.12, d)))
            let a = CGPoint(x: p.x - box.size.width / 2, y: p.y - box.size.height / 2)
            let r = z / zoom
            let o = CGSize(width: a.x - (a.x - offset.width) * r, height: a.y - (a.y - offset.height) * r)
            zoom = z
            offset = clamp(o, box.size, z)
            return nil
        }
    }

    private func removeScroll() {
        if let m = box.monitor { NSEvent.removeMonitor(m) }
        box.monitor = nil
    }

    // MARK: gestures

    private func magnify(_ geo: GeometryProxy, _ fit: CGFloat) -> some Gesture {
        MagnifyGesture()
            .onChanged { v in
                if pinchBase == nil { pinchBase = (zoom, offset) }
                guard let base = pinchBase else { return }
                let z = min(maxZoom, max(1, base.zoom * v.magnification))
                // keep the point under the fingers where it is
                let a = CGPoint(x: (v.startAnchor.x - 0.5) * geo.size.width, y: (v.startAnchor.y - 0.5) * geo.size.height)
                let r = z / base.zoom
                let o = CGSize(width: a.x - (a.x - base.offset.width) * r, height: a.y - (a.y - base.offset.height) * r)
                zoom = z
                offset = clamp(o, geo.size, z)
            }
            .onEnded { _ in pinchBase = nil }
    }

    private func pan(_ geo: GeometryProxy, _ fit: CGFloat) -> some Gesture {
        DragGesture(minimumDistance: 4)
            .onChanged { v in
                guard zoom > 1.001 else { return }
                if dragBase == nil { dragBase = offset }
                guard let base = dragBase else { return }
                offset = clamp(CGSize(width: base.width + v.translation.width, height: base.height + v.translation.height), geo.size, zoom)
            }
            .onEnded { _ in dragBase = nil }
    }

    /// Don't let the drawing slide out of view.
    private func clamp(_ o: CGSize, _ view: CGSize, _ z: CGFloat) -> CGSize {
        let s = stage
        let fit = max(0.05, min(view.width / s.width, view.height / s.height))
        let w = s.width * fit * z, h = s.height * fit * z
        let mx = max(0, (w - view.width) / 2 + 40), my = max(0, (h - view.height) / 2 + 40)
        return CGSize(width: min(mx, max(-mx, o.width)), height: min(my, max(-my, o.height)))
    }

    private func step(_ factor: CGFloat, _ geo: GeometryProxy, _ fit: CGFloat) {
        let z = min(maxZoom, max(1, zoom * factor))
        let r = z / zoom
        withAnimation(.easeOut(duration: 0.18)) {
            zoom = z
            offset = clamp(CGSize(width: offset.width * r, height: offset.height * r), geo.size, z)
        }
    }

    // MARK: controls

    private func controls(_ geo: GeometryProxy, _ fit: CGFloat) -> some View {
        HStack(spacing: 2) {
            Button { step(1 / 1.25, geo, fit) } label: { Image(systemName: "minus.magnifyingglass") }
                .keyboardShortcut("-", modifiers: .command)
                .disabled(zoom <= 1.001)
                .help("Zoom out (⌘−)")
            Button {
                withAnimation(.easeOut(duration: 0.2)) { zoom = 1; offset = .zero }
            } label: {
                Text("\(Int((zoom * 100).rounded()))%").font(Theme.font(11, .semibold, .mono))
                    .frame(minWidth: 40)
            }
            .keyboardShortcut("0", modifiers: .command)
            .help("Fit to window (⌘0)")
            Button { step(1.25, geo, fit) } label: { Image(systemName: "plus.magnifyingglass") }
                .keyboardShortcut("=", modifiers: .command)
                .disabled(zoom >= maxZoom - 0.001)
                .help("Zoom in (⌘+). Pinch on the trackpad to zoom where you point; drag to move around.")
        }
        .buttonStyle(.borderless)
        .foregroundStyle(Theme.ink)
        .padding(.horizontal, 8).padding(.vertical, 5)
        .background(Capsule().fill(Theme.panel.opacity(0.92)))
        .overlay(Capsule().stroke(Theme.line, lineWidth: 1))
    }
}

/// Natural size of a growing stage's content.
private struct NaturalSizeKey: PreferenceKey {
    static let defaultValue: CGSize = .zero
    static func reduce(value: inout CGSize, nextValue: () -> CGSize) {
        let n = nextValue()
        value = CGSize(width: max(value.width, n.width), height: max(value.height, n.height))
    }
}

extension Theme {
    /// Text-size factor for drawings: boxes that hold text grow with it (never shrink below the design size).
    static var grow: CGFloat { CGFloat(max(1, Appearance.shared.data.scale)) }
}
