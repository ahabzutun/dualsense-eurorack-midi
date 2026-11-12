// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "DualSenseMapper",
    platforms: [.macOS(.v14)],
    targets: [
        .executableTarget(name: "DualSenseMapper", path: "Sources/DualSenseMapper")
    ]
)
