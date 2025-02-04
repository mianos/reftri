## tests
Here are the `curl` commands to set the **pump duty cycle**, **inversion**, and **frequency (max speed)** for the **TMCWebServer** device.

---

### **1. Set Pump Duty Cycle**
To set the pump **duty cycle** (e.g., **50%**):
```sh
curl -X POST http://${ESP_IP}/pump -H "Content-Type: application/json" -d '{"duty": 50.0}'
```
- `"duty"` is a percentage between **0.0 and 100.0**.
- The actual motor speed is derived from `max_speed` and will be inverted if `invert` is enabled.

---

### **2. Set Inversion**
To **enable** inversion:
```sh
curl -X POST http://${ESP_IP}/settings -H "Content-Type: application/json" -d '{"invert": true}'
```
To **disable** inversion:
```sh
curl -X POST http://${ESP_IP}/settings -H "Content-Type: application/json" -d '{"invert": false}'
```

---

### **3. Set Frequency (Max Speed)**
To set the **maximum motor speed** (e.g., **5000** units):
```sh
curl -X POST http://${ESP_IP}/settings -H "Content-Type: application/json" -d '{"max_speed": 5000}'
```
- This value determines the full-speed limit of the motor.
- The duty cycle percentage scales the actual speed.

---

### **4. Set Both Inversion and Max Speed**
To update **both** inversion and max speed in one request:
```sh
curl -X POST http://${ESP_IP}/settings -H "Content-Type: application/json" -d '{"invert": true, "max_speed": 5000}'
```

---

### **5. Set Motor Speed Directly**
To manually set the **motor speed** (bypassing duty cycle logic):
```sh
curl -X POST http://${ESP_IP}/set_motor_speed -H "Content-Type: application/json" -d '{"speed": 2500}'
```
- Directly sets the motor velocity without using the **duty cycle percentage**.

---

#### **Notes**
- `${ESP_IP}` should be replaced with the actual device IP.
- `"invert"` is **true** or **false**.
- `"max_speed"` must be a **positive integer**.
- `"duty"` is a **float (0.0 - 100.0%)**.
- If **inversion is enabled**, duty cycle is flipped (`100% → 0%` and `0% → 100%`).
