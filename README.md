# Arduino Nano RP2040 Connect Flight Controller

<div align=center><img src=https://user-images.githubusercontent.com/38836869/200135663-ea6b8495-fe91-43e7-9829-e90665b17b86.png width=400></div>

## Introduction

The Arduino Nano RP2040 Connect comes with a built in IMU. It's fast processing speed and ability to program it with the Arduino IDE makes for a great microcontroller to serve as a quadcopter (drone) flight controller.

The code in the repository is MIT licensed open source.  Do whatever you like with it. If you'd like to simply contribute to this source, perform a pull request and get your 'git' on.  We'd appreciate it.

We have designed a printed circuit board (PCB) that goes along with this code.  It allows hookups to beefy motors and their ESCs.  You can find it here: 
[Arduino Nano RP2040 Connect Flight Controller PCB](https://www.pcbway.com/project/shareproject/First_Opensource_Arduino_Nano_RP2040_Connect_Drone_Flight_Controller_4f90bf4e.html)

What's special about this project is that at the time of writing this article, there are no search results via Google, Bing, or Yahoo that can find a published opensource code for the Arduino Nano RP2040 Connect for flight control. This makes this project the first published open-source version!

## Attribution
We got our starter code from nickrehm's repository dRehmFlight.  It is great IMU focused code for the Teensy that we ported for our hardware and to pursue additional features such as proximity sensors and GPS.  You can find his repository here: [dRehmFlight](https://github.com/nickrehm/dRehmFlight)
## Hardware
These are the components we are using for our drone:

- Microcontroller:  Arduino Nano RP2040 Connect
- Motors: QWinOut A2212 1000KV Brushless Outrunner Motor 13T
- ESCs: QWinOut 2-4S 30A/40A RC Brushless ESC Simonk Firmware Electric Speed Controller with 5V 3A BEC
- Battery: HRB 4S 3300mAh 14.8v Lipo RC Battery 60C

This hardware works with the code out of the box.  The drone frame is up to the builder.

## PCB Design Considerations

The biggest design consideration for this board was the logic level. The RP2040 GPIO is designed for 3.3V and not 5V tolerant. However, the ESCs and the radio receiver communicate on 5V.  So, any pin to the RP2040 that was receiving a signal required a voltage divider to protect the RP2040.

The second consideration was battery management. Many of the LiPo batteries available are long lasting with a ton of amps to deliver quickly, but must not be allowed to go under their minimum cell voltage. We balooned a LiPo once in the house.  It's terrifying!

In turn, a means of letting the drone pilot know the battery is draining too low had to be designed.  We achieved this by applying a voltage divider here as well. In our case, we had a 14.8V divided to just under 3Vs. The closer the battery approaches undervoltage (12.8V), the beeper will increase its beep rate until it is too annoying to want to continue use.

Otherwise, the rest of the design was just focused on providing reliable wire hookups between the drone receiver and motor electronic speed controllers (ESCs).

# Tuning
<hr><h3><b>Ziegler-Nichols</b></h3>  <p>The Ziegler-Nichols tuning method is one of the most famous ways to experimentally tune a PID controller. The basic algorithm is as follows:</p><ol><li>Turn off the Integral and Derivative components for the controller; only use Proportional control.</li><li>Slowly increase the gain (i.e. <em>K<sub>p</sub></em>, the Proportion constant) until the process starts to oscillate<br />This final gain value is known as the ultimate gain, or <em>K<sub>u</sub></em><br />The period of oscillation is the ultimate period, or <em>T<sub>u</sub></em></li><li>Use the following table to derive the PID variables</li></ol></div></div><div class=\"sqs-block code-block sqs-block-code\" data-block-type=\"23\" id=\"block-yui_3_17_2_1_1598301478634_207710\"><div class=\"sqs-block-content\"><style type=\"text/css\">.tg  {border-collapse:collapse;border-spacing:0;}.tg td{border-color:black;border-style:solid;border-width:1px;  overflow:hidden;padding:10px 5px;word-break:normal;}.tg th{border-color:black;border-style:solid;border-width:1px;  font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}.tg .tg-gr1d{background-color:#e9f4fc;border-color:#337494;font-weight:bold;text-align:left;vertical-align:top}.tg .tg-m4ds{border-color:#337494;text-align:center;vertical-align:top}.tg .tg-bxxa{border-color:#337494;text-align:left;vertical-align:top}.tg .tg-hk19{background-color:#e9f4fc;border-color:#337494;font-weight:bold;text-align:center;vertical-align:top}.tg .tg-vaq8{background-color:#e9f4fc;border-color:#337494;text-align:left;vertical-align:top}.tg .tg-e1cu{background-color:#e9f4fc;border-color:#337494;text-align:center;vertical-align:top}</style><table class=\"table\"><thead>  <tr>    <th class=\"tg-gr1d\">Controller</th>    <th class=\"tg-hk19\">K<sub>p</sub></th>    <th class=\"tg-hk19\">T<sub>i</sub></th>    <th class=\"tg-hk19\">T<sub>d</sub></th>  </tr></thead><tbody>  <tr>    <td class=\"tg-bxxa\">P</td>    <td class=\"tg-m4ds\">K<sub>u</sub>/2</td>    <td class=\"tg-m4ds\"></td>    <td class=\"tg-m4ds\"></td>  </tr>  <tr>    <td class=\"tg-vaq8\">PI</td>    <td class=\"tg-e1cu\">K<sub>u</sub>/2.5</td>    <td class=\"tg-e1cu\">T<sub>u</sub>/1.25</td>    <td class=\"tg-e1cu\"></td>  </tr>  <tr>    <td class=\"tg-bxxa\">PID</td>    <td class=\"tg-m4ds\">0.6K<sub>u</sub></td>    <td class=\"tg-m4ds\">T<sub>u</sub>/2</td>    <td class=\"tg-m4ds\">T<sub>u</sub>/8</td>  </tr>  <tr>    <td class=\"tg-vaq8\">Pessen Integral Rule<br></td>    <td class=\"tg-e1cu\">0.7K<sub>u</sub></td>    <td class=\"tg-e1cu\">0.4T<sub>u</sub></td>    <td class=\"tg-e1cu\">0.15T<sub>u</sub></td>  </tr>  <tr>    <td class=\"tg-bxxa\">Moderate overshoot</td>    <td class=\"tg-m4ds\">K<sub>u</sub>/3</td>    <td class=\"tg-m4ds\">T<sub>u</sub>/2</td>    <td class=\"tg-m4ds\">T<sub>u</sub>/3</td>  </tr>  <tr>    <td class=\"tg-vaq8\">No overshoot</td>    <td class=\"tg-e1cu\">K<sub>u</sub>/5</td>    <td class=\"tg-e1cu\">T<sub>u</sub>/2</td>    <tdclass=\"tg-e1cu\">T<sub>u</sub>/3</td>  </tr></tbody></table>

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
