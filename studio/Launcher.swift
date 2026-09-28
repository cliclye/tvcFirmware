import AppKit

final class StudioDelegate: NSObject, NSApplicationDelegate {
    private var process: Process?
    private var studioURL: URL?
    private var output = Data()
    private var startupTimer: Timer?

    func applicationDidFinishLaunching(_ notification: Notification) {
        let menu = NSMenu()
        let item = NSMenuItem()
        let submenu = NSMenu()
        submenu.addItem(withTitle: "Open EasyTVC Studio", action: #selector(openStudio), keyEquivalent: "o").target = self
        submenu.addItem(NSMenuItem.separator())
        submenu.addItem(withTitle: "Quit EasyTVC Studio", action: #selector(NSApplication.terminate(_:)), keyEquivalent: "q")
        item.submenu = submenu
        menu.addItem(item)
        NSApp.mainMenu = menu
        guard let resource = Bundle.main.resourceURL else { fail("App resources are missing."); return }
        let paths = ["/opt/homebrew/bin/python3", "/usr/local/bin/python3", "/usr/bin/python3"]
        guard let python = paths.first(where: { FileManager.default.isExecutableFile(atPath: $0) }) else {
            fail("Python 3.9 or newer is needed. Install Python, then reopen Studio."); return
        }
        let child = Process()
        child.executableURL = URL(fileURLWithPath: python)
        child.arguments = [resource.appendingPathComponent("studio.py").path, "--no-browser"]
        child.currentDirectoryURL = resource
        let pipe = Pipe()
        child.standardOutput = pipe
        child.standardError = pipe
        pipe.fileHandleForReading.readabilityHandler = { [weak self] handle in
            let data = handle.availableData
            if data.isEmpty { handle.readabilityHandler = nil; return }
            DispatchQueue.main.async {
                guard let self = self else { return }
                self.output.append(data)
                if self.output.count > 16384 { self.output = self.output.suffix(16384) }
                let text = String(data: self.output, encoding: .utf8) ?? ""
                if self.studioURL == nil, let line = text.components(separatedBy: "\n").first(where: { $0.hasPrefix("EasyTVC Studio: http://127.0.0.1:") }),
                   let url = URL(string: String(line.dropFirst("EasyTVC Studio: ".count))) {
                    self.studioURL = url
                    self.startupTimer?.invalidate()
                    self.openStudio()
                }
            }
        }
        child.terminationHandler = { [weak self] process in
            DispatchQueue.main.async {
                if process.terminationStatus != 0 {
                    let diagnostic = String(data: self?.output ?? Data(), encoding: .utf8) ?? ""
                    self?.fail("The local service stopped.\n\(diagnostic)")
                } else { NSApp.terminate(nil) }
            }
        }
        process = child
        do { try child.run() } catch { fail(error.localizedDescription); return }
        startupTimer = Timer.scheduledTimer(withTimeInterval: 15, repeats: false) { [weak self] _ in
            self?.fail("The local service did not start within 15 seconds.")
        }
    }

    @objc func openStudio() { if let url = studioURL { NSWorkspace.shared.open(url) } }
    func applicationShouldHandleReopen(_ sender: NSApplication, hasVisibleWindows flag: Bool) -> Bool { openStudio(); return true }
    func applicationWillTerminate(_ notification: Notification) {
        startupTimer?.invalidate()
        process?.terminationHandler = nil
        if process?.isRunning == true { process?.terminate() }
    }
    private func fail(_ message: String) {
        startupTimer?.invalidate()
        let alert = NSAlert()
        alert.messageText = "EasyTVC Studio could not open"
        alert.informativeText = message
        alert.runModal()
        NSApp.terminate(nil)
    }
}

let app = NSApplication.shared
let delegate = StudioDelegate()
app.delegate = delegate
app.setActivationPolicy(.regular)
app.run()
