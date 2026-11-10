import CoreBluetooth
import Foundation

/// Bluetooth LE link to a DroneBridge for ESP32 in BLE mode, relayed to Studio's ground link as UDP
/// on 127.0.0.1 (Python's standard library has no Bluetooth; CoreBluetooth is built into macOS).
///
/// - Studio sends to 127.0.0.1:14570 (its keep-alive PING arrives every second). While those keep
///   coming, the relay scans for a DroneBridge, connects and passes bytes both ways unchanged:
///   notifications from characteristic DB34 go to Studio, Studio's datagrams are written to DB33.
/// - Once Studio has been quiet for a few seconds the relay disconnects and stops scanning.
/// - Once a second it tells Studio how the radio side is doing, as a "#BLE ..." line (not an E1
///   frame: the ground link shows it and never treats it as telemetry).
/// The relay never makes up or changes data: what the board accepts is still decided by the board.
final class BleRelay: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    static let port: UInt16 = 14570
    private let service = CBUUID(string: "DB32")       // 0000db32-0000-1000-8000-00805f9b34fb
    private let txChar = CBUUID(string: "DB33")        // write: ground -> board
    private let rxChar = CBUUID(string: "DB34")        // notify: board -> ground
    private let queue = DispatchQueue(label: "easytvc.ble")
    private var central: CBCentralManager?   // created on first use: macOS asks for Bluetooth permission then
    private var peripheral: CBPeripheral?
    private var tx: CBCharacteristic?
    private var socket: Int32 = -1
    private var readSource: DispatchSourceRead?
    private var timer: DispatchSourceTimer?
    private var client: sockaddr_in?
    private var lastClient = Date.distantPast
    private var state = "idle"
    private var rssi: Int?
    private var name = ""

    func start() {
        queue.async { [self] in
            guard self.openSocket() else { return }
            let timer = DispatchSource.makeTimerSource(queue: self.queue)
            timer.schedule(deadline: .now() + 1, repeating: 1)
            timer.setEventHandler { [weak self] in self?.tick() }
            timer.resume()
            self.timer = timer
        }
    }

    // MARK: UDP on 127.0.0.1 only
    private func openSocket() -> Bool {
        socket = Darwin.socket(AF_INET, SOCK_DGRAM, 0)
        guard socket >= 0 else { return false }
        var address = sockaddr_in()
        address.sin_len = UInt8(MemoryLayout<sockaddr_in>.size)
        address.sin_family = sa_family_t(AF_INET)
        address.sin_port = BleRelay.port.bigEndian
        address.sin_addr.s_addr = inet_addr("127.0.0.1")
        let bound = withUnsafePointer(to: &address) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) { Darwin.bind(socket, $0, socklen_t(MemoryLayout<sockaddr_in>.size)) }
        }
        guard bound == 0 else { Darwin.close(socket); socket = -1; return false }   // another Studio has it
        let source = DispatchSource.makeReadSource(fileDescriptor: socket, queue: queue)
        source.setEventHandler { [weak self] in self?.receive() }
        source.resume()
        readSource = source
        return true
    }

    private func receive() {
        var buffer = [UInt8](repeating: 0, count: 2048)
        var from = sockaddr_in()
        var length = socklen_t(MemoryLayout<sockaddr_in>.size)
        let n = withUnsafeMutablePointer(to: &from) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) { recvfrom(socket, &buffer, buffer.count, 0, $0, &length) }
        }
        guard n > 0 else { return }
        client = from
        lastClient = Date()
        if central == nil { central = CBCentralManager(delegate: self, queue: queue) }
        if peripheral == nil && central?.isScanning != true {
            describeCentral()                         // waking up again: report the radio as it is now
            if central?.state == .poweredOn { scan() }
        }
        guard let peripheral = peripheral, let tx = tx else { return }
        // write in pieces the link can carry; frames are LF-terminated, so splitting is harmless
        let withoutResponse = tx.properties.contains(.writeWithoutResponse)
        let chunk = max(20, peripheral.maximumWriteValueLength(for: withoutResponse ? .withoutResponse : .withResponse))
        var offset = 0
        while offset < n {
            let end = min(n, offset + chunk)
            peripheral.writeValue(Data(buffer[offset..<end]), for: tx, type: withoutResponse ? .withoutResponse : .withResponse)
            offset = end
        }
    }

    private func send(_ data: Data) {
        guard var to = client, socket >= 0 else { return }
        _ = data.withUnsafeBytes { bytes in
            withUnsafePointer(to: &to) {
                $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                    sendto(socket, bytes.baseAddress, data.count, 0, $0, socklen_t(MemoryLayout<sockaddr_in>.size))
                }
            }
        }
    }

    private func tick() {
        let active = Date().timeIntervalSince(lastClient) < 5
        if !active {
            if let p = peripheral { central?.cancelPeripheralConnection(p) }
            if central?.isScanning == true { central?.stopScan() }
            if state != "idle" { state = "idle" }
            return
        }
        peripheral?.readRSSI()
        var line = "#BLE state=\(state)"
        if !name.isEmpty { line += " name=\(name.replacingOccurrences(of: " ", with: "_"))" }
        if let r = rssi { line += " rssi=\(r)" }
        send(Data((line + "\n").utf8))
    }

    // MARK: CoreBluetooth
    private func scan() {
        guard let central = central, central.state == .poweredOn, !central.isScanning else { return }
        state = "scanning"
        central.scanForPeripherals(withServices: nil, options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        describeCentral()
        if central.state == .poweredOn && Date().timeIntervalSince(lastClient) < 5 { scan() }
    }

    /// The radio's own state, for when no device is being looked for or talked to.
    private func describeCentral() {
        switch central?.state {
        case .poweredOn?: state = "ready"
        case .unauthorized?: state = "not_allowed"     // System Settings > Privacy & Security > Bluetooth
        case .poweredOff?: state = "bluetooth_off"
        case .unsupported?: state = "unsupported"
        default: state = "starting"                   // includes waiting for the permission answer
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        let services = advertisementData[CBAdvertisementDataServiceUUIDsKey] as? [CBUUID] ?? []
        let local = (advertisementData[CBAdvertisementDataLocalNameKey] as? String) ?? peripheral.name ?? ""
        guard services.contains(service) || local.lowercased().contains("dronebridge") else { return }
        central.stopScan()
        self.peripheral = peripheral
        name = local.isEmpty ? "DroneBridge" : local
        rssi = RSSI.intValue
        state = "connecting"
        peripheral.delegate = self
        central.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        state = "discovering"
        peripheral.discoverServices([service])
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        forget()
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        forget()       // the next datagram from Studio starts a new scan
    }

    private func forget() {
        peripheral = nil
        tx = nil
        rssi = nil
        describeCentral()
        if Date().timeIntervalSince(lastClient) < 5 { scan() }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let s = peripheral.services?.first(where: { $0.uuid == service }) else {
            state = "wrong_device"; central?.cancelPeripheralConnection(peripheral); return
        }
        peripheral.discoverCharacteristics([txChar, rxChar], for: s)
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        for c in service.characteristics ?? [] {
            if c.uuid == txChar { tx = c }
            if c.uuid == rxChar { peripheral.setNotifyValue(true, for: c) }
        }
        state = tx == nil ? "wrong_device" : "connected"
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard characteristic.uuid == rxChar, let value = characteristic.value, !value.isEmpty else { return }
        send(value)
    }

    func peripheral(_ peripheral: CBPeripheral, didReadRSSI RSSI: NSNumber, error: Error?) {
        if error == nil { rssi = RSSI.intValue }
    }
}
