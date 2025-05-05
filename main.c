/* Tiny Neon Inverter
 * -------------------
 * See schematics for circuit
 * Can generate 120-330V DC from 5V supply
 *
 * Pinout:
 * - Pin 1: ADCO - Voltage divider ADC feedback input
 * - Pin 3: OC0B - PWM output to gate driver
 *
 * THIS CODE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Luigi Pizzolito, 2025
 */

#include <avr/io.h>
#include <util/delay.h>
#define F_CPU 8000000UL

// 40-120kHz is good
// Limit duty cycle to 70.5%
//
// Target Voltage 180V
// divider = 100:1
// ADC range 0-255
// Target ADC = 180V / 100 = 1.8V
// ADC = 1.8V * 255 / 5V = 91.8
#define ADC_TARGET (uint8_t)92
#define DUTY_TOP (uint8_t)80
#define DUTY_MAX (uint8_t)40

void clock_init() {
  // Set system clock 8MHz, internal, prescaler 1
  CCP = 0xD8;
  CLKMSR = 0x00;
  CCP = 0xD8;
  CLKPSR = 0x00;
}

void adc_init() {
  // Set PB0 as input
  DDRB &= ~(1 << PB0);
  // Setup ADC
  ADMUX = 0x00;                        // ADC0, Vcc reference
  ADCSRA = (1 << ADEN) | (1 << ADPS0); // Enable ADC, 125kHz clock
}

uint16_t adc_read() {
  ADCSRA |= (1 << ADSC); // Start conversion
  while (ADCSRA & (1 << ADSC))
    ; // Wait for conversion to finish
  return ADCL;
}

void pwm_init() {
  // Fast PWM on OC0B (PB1) (0-80) 95kHz, inverted
  /*TCCR0A = (2 << COM0B0) | (3 << WGM00);*/
  TCCR0A = _BV(COM0B1) | _BV(COM0B0) | _BV(WGM01);
  TCCR0B = _BV(CS00) | _BV(WGM03) | _BV(WGM02);
  /*TCCR0B = (0 << WGM02) | (1 << CS00); // No prescaler*/
  ICR0 = DUTY_TOP;
  OCR0B = ICR0;       // Set initial duty cycle to 0
  DDRB |= (1 << PB1); // Set OC0B as output
}

uint8_t duty = 0;

int main() {
  clock_init();
  adc_init();
  pwm_init();

  // OCR0B = 80 - 56;

  while (1) {
    // Read ADC value
    uint8_t adc = adc_read();
    // Set duty cycle based on ADC value
    if (adc < ADC_TARGET && duty < DUTY_MAX) {
      duty++;
    } else if (adc >= ADC_TARGET && duty > 0) {
      duty--;
    }
    OCR0B = ICR0 - duty; // Set duty cycle
    _delay_ms(2);        // 100Hz loop rate
  }
}
