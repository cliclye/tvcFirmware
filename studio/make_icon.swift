// Draws the EasyTVC Studio app icon (purple tile, white rocket arrow) into an .iconset folder.
import AppKit

let out = CommandLine.arguments[1]
try? FileManager.default.createDirectory(atPath: out, withIntermediateDirectories: true)
for (size, scale) in [(16, 1), (16, 2), (32, 1), (32, 2), (128, 1), (128, 2), (256, 1), (256, 2), (512, 1), (512, 2)] {
    let px = size * scale
    let rep = NSBitmapImageRep(bitmapDataPlanes: nil, pixelsWide: px, pixelsHigh: px, bitsPerSample: 8,
                               samplesPerPixel: 4, hasAlpha: true, isPlanar: false, colorSpaceName: .deviceRGB,
                               bytesPerRow: 0, bitsPerPixel: 0)!
    NSGraphicsContext.saveGraphicsState()
    NSGraphicsContext.current = NSGraphicsContext(bitmapImageRep: rep)
    let s = CGFloat(px), inset = s * 0.09
    let tile = NSBezierPath(roundedRect: NSRect(x: inset, y: inset, width: s - 2 * inset, height: s - 2 * inset),
                            xRadius: s * 0.2, yRadius: s * 0.2)
    NSGradient(starting: NSColor(red: 0.47, green: 0.36, blue: 0.94, alpha: 1),
               ending: NSColor(red: 0.33, green: 0.22, blue: 0.75, alpha: 1))!.draw(in: tile, angle: -60)
    let glyph = NSAttributedString(string: "↗", attributes: [
        .font: NSFont.systemFont(ofSize: s * 0.58, weight: .semibold), .foregroundColor: NSColor.white])
    let g = glyph.size()
    glyph.draw(at: NSPoint(x: (s - g.width) / 2, y: (s - g.height) / 2))
    NSGraphicsContext.restoreGraphicsState()
    let name = scale == 1 ? "icon_\(size)x\(size).png" : "icon_\(size)x\(size)@2x.png"
    try! rep.representation(using: .png, properties: [:])!.write(to: URL(fileURLWithPath: out + "/" + name))
}
