
struct INA181<'a> {
    unit: Float,
    adc: AdcContDriver<'a>,
}
impl INA181 {
    fn new(sense_amplifier: Float, sense_resistance: Float, reference_voltage: Float) -> Self {
        Self {
            unit: sense_amplifier * reference_voltage / sense_resistance,
            adc: AdcContDriver::new(peripheral
        }
    }
    fn current(&mut self) -> Float {
        let measure = todo!("adc conversion");
        measure * self.unit
    }
}
