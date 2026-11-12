import SwiftUI

// MARK: - Look

enum Theme {
    // colours and fonts come from Appearance (adjustable, see the paintbrush button)
    static var bg: Color { Appearance.shared.color(.bg) }
    static var panel: Color { Appearance.shared.color(.panel) }
    static var card: Color { Appearance.shared.color(.card) }
    static var line: Color { Appearance.shared.color(.line) }
    static var ink: Color { Appearance.shared.color(.ink) }
    static var dim: Color { Appearance.shared.color(.dim) }
    static var body: Color { Appearance.shared.color(.body) }         // schematic line colour
    static var cv: Color { Appearance.shared.color(.cv) }
    static var trig: Color { Appearance.shared.color(.trig) }
    static var special: Color { Appearance.shared.color(.special) }
    static var octave: Color { Appearance.shared.color(.octave) }
    static var function: Color { Appearance.shared.color(.function) } // controller functions (loop, motion …)
    static var note: Color { Appearance.shared.color(.note) }
    static var field: Color { Appearance.shared.fieldFill }          // number / text field background

    static func font(_ size: CGFloat, _ weight: Font.Weight = .regular, _ role: Appearance.FontRole = .ui) -> Font {
        Appearance.shared.font(size, weight, role)
    }

    static func color(_ r: Role) -> Color {
        switch r {
        case .off: return dim
        case .cv: return cv
        case .trigger, .noteTrigger: return trig
        case .lfoRate, .lfoReset: return special
        case .octave: return octave
        }
    }
    static func color(_ k: Mapping.TokenKind) -> Color {
        switch k {
        case .off: return dim
        case .route(let r): return color(r)
        case .function: return function
        }
    }
}

// MARK: - Geometry: the controller is drawn in its own 900 × 600 space, placed into 1100 × 640

enum Geo {
    static let canvas = CGSize(width: 1100, height: 640)
    static let s: CGFloat = 0.80
    static let cx: CGFloat = 550, cy: CGFloat = 330
    static func p(_ x: CGFloat, _ y: CGFloat) -> CGPoint { CGPoint(x: cx + (x - 450) * s, y: cy + (y - 290) * s) }
    static func d(_ v: CGFloat) -> CGFloat { v * s }
    static var transform: CGAffineTransform {
        CGAffineTransform(translationX: cx - 450 * s, y: cy - 290 * s).scaledBy(x: s, y: s)
    }
}

struct BodyShape: Shape {
    func path(in rect: CGRect) -> Path {
        var p = Path()
        p.move(to: CGPoint(x: 300, y: 112))
        p.addQuadCurve(to: CGPoint(x: 600, y: 112), control: CGPoint(x: 450, y: 96))
        p.addCurve(to: CGPoint(x: 762, y: 160), control1: CGPoint(x: 682, y: 104), control2: CGPoint(x: 737, y: 120))
        p.addCurve(to: CGPoint(x: 838, y: 432), control1: CGPoint(x: 803, y: 222), control2: CGPoint(x: 845, y: 342))
        p.addCurve(to: CGPoint(x: 746, y: 522), control1: CGPoint(x: 834, y: 500), control2: CGPoint(x: 792, y: 532))
        p.addCurve(to: CGPoint(x: 640, y: 410), control1: CGPoint(x: 702, y: 512), control2: CGPoint(x: 682, y: 432))
        p.addCurve(to: CGPoint(x: 260, y: 410), control1: CGPoint(x: 560, y: 386), control2: CGPoint(x: 340, y: 386))
        p.addCurve(to: CGPoint(x: 154, y: 522), control1: CGPoint(x: 218, y: 432), control2: CGPoint(x: 198, y: 512))
        p.addCurve(to: CGPoint(x: 62, y: 432), control1: CGPoint(x: 108, y: 532), control2: CGPoint(x: 66, y: 500))
        p.addCurve(to: CGPoint(x: 138, y: 160), control1: CGPoint(x: 55, y: 342), control2: CGPoint(x: 97, y: 222))
        p.addCurve(to: CGPoint(x: 300, y: 112), control1: CGPoint(x: 163, y: 120), control2: CGPoint(x: 218, y: 104))
        p.closeSubpath()
        return p.applying(Geo.transform)
    }
}

// MARK: - Canvas

@MainActor
struct ControllerCanvas: View {
    @Environment(Mapping.self) private var mapping
    @Binding var selected: GroupID

    var body: some View {
        ZoomStage(size: Geo.canvas) {
            ZStack(alignment: .topLeading) {
                BlueprintGrid()
                BodyShape()
                    .fill(LinearGradient(colors: [Theme.body.opacity(0.10), Theme.body.opacity(0.03)],
                                         startPoint: .top, endPoint: .bottom))
                BodyShape().stroke(Theme.body.opacity(0.55), lineWidth: 1.6)
                BodyShape().stroke(Theme.body.opacity(0.12), lineWidth: 7).blur(radius: 6)

                decorations
                callouts
                controls
            }
        }
    }

    private var decorations: some View {
        ZStack(alignment: .topLeading) {
            Text("CREATE").font(Theme.font(8, .semibold, .mono)).foregroundStyle(Theme.dim)
                .position(Geo.p(302, 178))
            Text("OPTIONS").font(Theme.font(8, .semibold, .mono)).foregroundStyle(Theme.dim)
                .position(Geo.p(598, 178))
        }
        .allowsHitTesting(false)
    }

    // MARK: mappable controls

    private func accent(_ g: GroupID) -> Color {
        for i in Catalog.group(g).inputs {
            for (_, k) in mapping.tokens(i) {
                if case .off = k { continue }
                return Theme.color(k)
            }
        }
        return Theme.body
    }
    private func mapped(_ g: GroupID) -> Bool {
        Catalog.group(g).inputs.contains { i in
            mapping.tokens(i).contains { t in if case .off = t.1 { return false } else { return true } }
        }
    }

    private var controls: some View {
        ZStack(alignment: .topLeading) {
            hot(.l2, RoundedRectangle(cornerRadius: Geo.d(15)), size: (130, 36), at: (212, 62), label: "L2")
            hot(.r2, RoundedRectangle(cornerRadius: Geo.d(15)), size: (130, 36), at: (688, 62), label: "R2")
            hot(.l1, RoundedRectangle(cornerRadius: Geo.d(11)), size: (118, 24), at: (212, 100), label: "L1")
            hot(.r1, RoundedRectangle(cornerRadius: Geo.d(11)), size: (118, 24), at: (688, 100), label: "R1")
            hot(.touchpad, RoundedRectangle(cornerRadius: Geo.d(26)), size: (236, 134), at: (450, 186), label: "TOUCHPAD")
            hot(.create, RoundedRectangle(cornerRadius: Geo.d(9)), size: (18, 36), at: (302, 142), label: "")
            hot(.options, RoundedRectangle(cornerRadius: Geo.d(9)), size: (18, 36), at: (598, 142), label: "")
            hot(.ps, Circle(), size: (34, 34), at: (450, 296), label: "PS")
            hot(.mic, Capsule(), size: (36, 14), at: (450, 332), label: "")
            dpad
            face(.triangle, "triangle", at: (688, 168))
            face(.circle, "circle", at: (734, 214))
            face(.cross, "xmark", at: (688, 260))
            face(.square, "square", at: (642, 214))
            stick(.leftStick, at: (334, 318))
            stick(.rightStick, at: (566, 318))
            centerTokens
            chip(.motion, icon: "gyroscope", text: "MOTION", at: (352, 470))
            chip(.slice, icon: "square.grid.3x3.fill", text: "GRAIN SLICE", at: (548, 470))
        }
    }

    /// PS and mic sit between the sticks: their tokens go right below them instead of a callout.
    private var centerTokens: some View {
        VStack(spacing: 3) {
            ForEach([GroupID.ps, .mic], id: \.self) { g in
                HStack(spacing: 5) {
                    Text(g == .ps ? "PS" : "MIC").font(Theme.font(8, .heavy, .mono))
                        .foregroundStyle(selected == g ? Theme.ink : Theme.dim)
                    TokenRow(input: Catalog.group(g).inputs[0], compact: true, showLabel: false)
                }
                .onTapGesture { selected = g }
            }
        }
        .position(Geo.p(450, 372))
    }

    private func hot<S: Shape>(_ g: GroupID, _ shape: S, size: (CGFloat, CGFloat), at c: (CGFloat, CGFloat), label: String) -> some View {
        let isSel = selected == g
        let col = accent(g)
        return Button { selected = g } label: {
            shape.fill(col.opacity(mapped(g) ? 0.16 : 0.04))
                .overlay(shape.stroke(isSel ? Theme.ink : col.opacity(mapped(g) ? 0.9 : 0.5), lineWidth: isSel ? 2.2 : 1.4))
                .overlay(Text(label).font(Theme.font(label.count > 2 ? 11 : 10, .bold, .mono))
                    .foregroundStyle(isSel ? Theme.ink : col.opacity(0.85)))
                .frame(width: Geo.d(size.0), height: Geo.d(size.1))
                .shadow(color: isSel ? col.opacity(0.8) : .clear, radius: 10)
                .contentShape(shape)
        }
        .buttonStyle(.plain)
        .position(Geo.p(c.0, c.1))
    }

    private var dpad: some View {
        let g = GroupID.dpad
        let isSel = selected == g
        let inputs = Catalog.group(g).inputs
        func col(_ i: InputDef) -> Color {
            let k = mapping.tokens(i).first?.1 ?? .off
            if case .off = k { return Theme.body }
            return Theme.color(k)
        }
        func on(_ i: InputDef) -> Bool { if case .off = mapping.tokens(i).first?.1 ?? .off { return false }; return true }
        let vCol = col(inputs[0]), hCol = col(inputs[1])
        let arm = Geo.d(34), len = Geo.d(108)
        return Button { selected = g } label: {
            ZStack {
                RoundedRectangle(cornerRadius: Geo.d(7)).fill(hCol.opacity(on(inputs[1]) ? 0.16 : 0.04))
                    .overlay(RoundedRectangle(cornerRadius: Geo.d(7)).stroke(isSel ? Theme.ink : hCol.opacity(0.75), lineWidth: isSel ? 2 : 1.3))
                    .frame(width: len, height: arm)
                RoundedRectangle(cornerRadius: Geo.d(7)).fill(vCol.opacity(on(inputs[0]) ? 0.16 : 0.04))
                    .overlay(RoundedRectangle(cornerRadius: Geo.d(7)).stroke(isSel ? Theme.ink : vCol.opacity(0.75), lineWidth: isSel ? 2 : 1.3))
                    .frame(width: arm, height: len)
                Image(systemName: "arrow.up.and.down.and.arrow.left.and.right")
                    .font(Theme.font(11, .semibold, .ui)).foregroundStyle(isSel ? Theme.ink : Theme.dim)
            }
            .frame(width: len, height: len)
            .shadow(color: isSel ? vCol.opacity(0.8) : .clear, radius: 10)
            .contentShape(Rectangle())
        }
        .buttonStyle(.plain)
        .position(Geo.p(212, 214))
    }

    private func face(_ g: GroupID, _ symbol: String, at c: (CGFloat, CGFloat)) -> some View {
        let isSel = selected == g
        let col = accent(g)
        let r = Geo.d(44)
        return Button { selected = g } label: {
            Circle().fill(col.opacity(mapped(g) ? 0.18 : 0.04))
                .overlay(Circle().stroke(isSel ? Theme.ink : col.opacity(mapped(g) ? 0.95 : 0.5), lineWidth: isSel ? 2.2 : 1.4))
                .overlay(Image(systemName: symbol).font(Theme.font(13, .bold, .ui))
                    .foregroundStyle(isSel ? Theme.ink : col.opacity(0.9)))
                .frame(width: r, height: r)
                .shadow(color: isSel ? col.opacity(0.8) : .clear, radius: 10)
                .contentShape(Circle())
        }
        .buttonStyle(.plain)
        .position(Geo.p(c.0, c.1))
    }

    private func stick(_ g: GroupID, at c: (CGFloat, CGFloat)) -> some View {
        let isSel = selected == g
        let col = accent(g)
        return Button { selected = g } label: {
            ZStack {
                Circle().fill(Theme.bg.opacity(0.6))
                Circle().stroke(col.opacity(mapped(g) ? 0.85 : 0.45), lineWidth: 1.3)
                Circle().fill(col.opacity(mapped(g) ? 0.18 : 0.05)).frame(width: Geo.d(66), height: Geo.d(66))
                Circle().stroke(isSel ? Theme.ink : col.opacity(0.8), lineWidth: isSel ? 2.2 : 1.3).frame(width: Geo.d(66), height: Geo.d(66))
                Circle().stroke(col.opacity(0.35), lineWidth: 1).frame(width: Geo.d(40), height: Geo.d(40))
                Path { p in
                    let m = Geo.d(50)
                    p.move(to: CGPoint(x: m - Geo.d(8), y: m)); p.addLine(to: CGPoint(x: m + Geo.d(8), y: m))
                    p.move(to: CGPoint(x: m, y: m - Geo.d(8))); p.addLine(to: CGPoint(x: m, y: m + Geo.d(8)))
                }
                .stroke(col.opacity(0.6), lineWidth: 1)
                .frame(width: Geo.d(100), height: Geo.d(100))
            }
            .frame(width: Geo.d(100), height: Geo.d(100))
            .shadow(color: isSel ? col.opacity(0.8) : .clear, radius: 12)
            .contentShape(Circle())
        }
        .buttonStyle(.plain)
        .position(Geo.p(c.0, c.1))
    }

    private func chip(_ g: GroupID, icon: String, text: String, at c: (CGFloat, CGFloat)) -> some View {
        let isSel = selected == g
        let col = accent(g)
        let group = Catalog.group(g)
        return Button { selected = g } label: {
            VStack(spacing: 5) {
                HStack(spacing: 6) {
                    Image(systemName: icon).font(Theme.font(11, .semibold, .ui))
                    Text(text).font(Theme.font(10, .bold, .mono))
                }
                .foregroundStyle(isSel ? Theme.ink : col.opacity(0.9))
                .padding(.horizontal, 12).padding(.vertical, 7)
                .background(Capsule().fill(col.opacity(mapped(g) ? 0.16 : 0.05)))
                .overlay(Capsule().stroke(isSel ? Theme.ink : col.opacity(0.7), style: StrokeStyle(lineWidth: isSel ? 2 : 1.2, dash: [4, 3])))
                .shadow(color: isSel ? col.opacity(0.8) : .clear, radius: 10)
                VStack(spacing: 2) {
                    ForEach(group.inputs) { i in
                        TokenRow(input: i, compact: true, showLabel: group.inputs.count > 1)
                    }
                }
            }
            .contentShape(Rectangle())
        }
        .buttonStyle(.plain)
        .position(Geo.p(c.0, c.1 + 26))
    }

    // MARK: callouts

    private struct Callout { let g: GroupID; let anchor: (CGFloat, CGFloat); let side: Side; let y: CGFloat }
    private enum Side { case left, right, top }

    private let callouts_: [Callout] = [
        Callout(g: .l2, anchor: (147, 62), side: .left, y: 66),
        Callout(g: .l1, anchor: (153, 100), side: .left, y: 124),
        Callout(g: .create, anchor: (294, 142), side: .left, y: 182),
        Callout(g: .dpad, anchor: (158, 214), side: .left, y: 258),
        Callout(g: .leftStick, anchor: (284, 318), side: .left, y: 380),
        Callout(g: .r2, anchor: (753, 62), side: .right, y: 66),
        Callout(g: .r1, anchor: (747, 100), side: .right, y: 124),
        Callout(g: .options, anchor: (606, 142), side: .right, y: 182),
        Callout(g: .triangle, anchor: (710, 168), side: .right, y: 236),
        Callout(g: .circle, anchor: (756, 214), side: .right, y: 280),
        Callout(g: .cross, anchor: (710, 260), side: .right, y: 324),
        Callout(g: .square, anchor: (642, 236), side: .right, y: 368),
        Callout(g: .rightStick, anchor: (616, 318), side: .right, y: 436),
        Callout(g: .touchpad, anchor: (450, 119), side: .top, y: 44),
    ]

    private var callouts: some View {
        ZStack(alignment: .topLeading) {
            ForEach(callouts_, id: \.g) { c in
                let a = Geo.p(c.anchor.0, c.anchor.1)
                let col = accent(c.g)
                let isSel = selected == c.g
                Path { p in
                    switch c.side {
                    case .left:
                        p.move(to: CGPoint(x: 214, y: c.y)); p.addLine(to: CGPoint(x: 244, y: c.y)); p.addLine(to: a)
                    case .right:
                        p.move(to: CGPoint(x: 886, y: c.y)); p.addLine(to: CGPoint(x: 856, y: c.y)); p.addLine(to: a)
                    case .top:
                        p.move(to: CGPoint(x: a.x, y: c.y + 26)); p.addLine(to: a)
                    }
                }
                .stroke(isSel ? Theme.ink.opacity(0.8) : col.opacity(mapped(c.g) ? 0.55 : 0.18),
                        style: StrokeStyle(lineWidth: isSel ? 1.4 : 1, dash: mapped(c.g) ? [] : [3, 4]))
                Circle().fill(isSel ? Theme.ink : col.opacity(mapped(c.g) ? 0.9 : 0.35)).frame(width: 5, height: 5).position(a)

                CalloutLabel(group: Catalog.group(c.g), selected: isSel, side: c.side == .left ? .trailing : c.side == .right ? .leading : .center)
                    .onTapGesture { selected = c.g }
                    .frame(width: c.side == .top ? 520 : 200, alignment: c.side == .left ? .trailing : c.side == .right ? .leading : .center)
                    .position(x: c.side == .left ? 110 : c.side == .right ? 990 : a.x, y: c.y)
            }
        }
    }
}

// MARK: - Labels

@MainActor
struct TokenRow: View {
    @Environment(Mapping.self) private var mapping
    let input: InputDef
    var compact = false
    var showLabel = true

    var body: some View {
        HStack(spacing: 5) {
            if showLabel {
                Text(input.short).font(Theme.font(compact ? 8 : 10, .semibold, .mono))
                    .foregroundStyle(Theme.dim)
            }
            ForEach(Array(mapping.tokens(input).enumerated()), id: \.offset) { _, t in
                let col = Theme.color(t.1)
                let off: Bool = { if case .off = t.1 { return true }; return false }()
                Text(t.0)
                    .font(Theme.font(compact ? 9 : 10.5, .bold, .mono))
                    .foregroundStyle(col)
                    .padding(.horizontal, off ? 0 : 6).padding(.vertical, 2)
                    .background(Capsule().fill(off ? Color.clear : col.opacity(0.14)))
            }
        }
        .fixedSize()
    }
}

@MainActor
struct CalloutLabel: View {
    @Environment(NotesStore.self) private var notes
    let group: GroupDef
    let selected: Bool
    let side: HorizontalAlignment

    var body: some View {
        VStack(alignment: side, spacing: 3) {
            Text(group.title.uppercased())
                .font(Theme.font(10, .heavy, .mono))
                .foregroundStyle(selected ? Theme.ink : Theme.ink.opacity(0.85))
                .tracking(1.2)
            if group.inputs.count == 1 {
                TokenRow(input: group.inputs[0], showLabel: false)
            } else if group.id == .touchpad {
                HStack(spacing: 14) { ForEach(group.inputs) { TokenRow(input: $0) } }
            } else {
                ForEach(group.inputs) { TokenRow(input: $0) }
            }
            let ns = group.inputs.compactMap { notes.note(NotesStore.ds($0.id)) }
            if let n = ns.first {
                NoteBadge(text: ns.joined(separator: " / "), size: 9.5)
                    .frame(maxWidth: group.id == .touchpad ? 480 : 190, alignment: side == .trailing ? .trailing : (side == .leading ? .leading : .center))
                    .id(n)
            }
        }
        .contentShape(Rectangle())
    }
}

struct BlueprintGrid: View {
    var body: some View {
        Canvas { ctx, size in
            var minor = Path(), major = Path()
            var x: CGFloat = 0
            while x <= size.width {
                let l = Path { $0.move(to: CGPoint(x: x, y: 0)); $0.addLine(to: CGPoint(x: x, y: size.height)) }
                if Int(x) % 100 == 0 { major.addPath(l) } else { minor.addPath(l) }
                x += 20
            }
            var y: CGFloat = 0
            while y <= size.height {
                let l = Path { $0.move(to: CGPoint(x: 0, y: y)); $0.addLine(to: CGPoint(x: size.width, y: y)) }
                if Int(y) % 100 == 0 { major.addPath(l) } else { minor.addPath(l) }
                y += 20
            }
            ctx.stroke(minor, with: .color(Theme.body.opacity(0.035)), lineWidth: 0.5)
            ctx.stroke(major, with: .color(Theme.body.opacity(0.07)), lineWidth: 0.7)
        }
        .allowsHitTesting(false)
    }
}
