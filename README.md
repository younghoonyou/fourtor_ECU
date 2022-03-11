# fourtor_ECU
2021 FOURTOR ECU TEAM
Hanyang Univ. ERICA Campus

https://www.notion.so/hoonworkspace/Electronic-Car-for-ECU-f8a141c54b9c46c8afb3294aa7ba8c90
<br>For more detail go to above link
<div align="center">
  
# Tech Stack
![C](https://img.shields.io/badge/C-A8B9CC?style=round-square&logo=C&logoColor=white) ![javascript](https://img.shields.io/badge/javascript-F7DF1E?&style=round-square&logo=javascript&logoColor=black) ![arduino](https://img.shields.io/badge/Arduino-00979D?style=round-square&logo=arduino&logoColor=white) ![stm32](https://img.shields.io/badge/STM32-03234B?style=round-square&logo=stmicroelectronics&logoColor=white) ![char.js](https://img.shields.io/badge/Chart.js-FF6384?style=round-square&logo=chart.js&logoColor=white) ![mysql](https://img.shields.io/badge/Mysql-4479A1?style=round-square&logo=mysql&logoColor=white) ![aws](https://img.shields.io/badge/AWS-232F3E?style=round-square&logo=amazonaws&logoColor=yellow) ![nginx](https://img.shields.io/badge/Nginx-009639?style=round-square&logo=nginx&logoColor=white)
    </div>
> # <div align="center">ECU</div>
<br>
<img src="https://user-images.githubusercontent.com/69233428/157842585-637dd3f1-1ec5-4d68-9f97-1a471d2b6c80.png">

#### Using STM32F446RE we made ECU for E-formula <br>
#### All data containing Motor,IMU,Battery passed by ECU to Driver 
<br>

> # <div align="center">Motor</div>
<br>
<div align="center">
<img src="https://user-images.githubusercontent.com/69233428/157976276-942bd076-78d8-4ae8-95f3-8e5bfd1adcaf.png">
<img src="https://user-images.githubusercontent.com/69233428/157976546-60a616b9-da1b-47eb-ae3a-d8178f8ef23d.png">
  </div>

#### Using Sevcon Gen4 controller read Motor's data including Torque,RPM,Motor's temperature
> # <div align="center">Battery
<br>
<p>
<img src="https://user-images.githubusercontent.com/69233428/157974028-46d2b951-8916-4dd5-b83e-055f85612c95.png" width="350" height="530">
<img src="https://user-images.githubusercontent.com/69233428/157974034-11da9c7a-4d50-4325-8956-6ff793e6545a.png" width="350" height="530">
<img src="https://user-images.githubusercontent.com/69233428/157977477-ea5622ba-3361-450b-b9c2-406e5aaef4ff.jpg" width="350" height="530">
  <div align="center">
<img src="https://user-images.githubusercontent.com/69233428/157975280-87a346ab-42f9-428b-8101-00b6e5161c36.gif">
    </div>
  </p>
  </div>
                                                                                                                 
#### From Battery and BMS logging battery's cell voltage and temperature
> # <div align="center">Driver</div>
<br>
<div align="center">
<img src="https://user-images.githubusercontent.com/69233428/157978207-5ae008dd-cc52-4201-820f-6d18599b8dc6.png" widh="900" height="700">
  </div>

#### Driver can view vehicle Data through LCD1602 and 7-segment
> # <div align="center">Telemetry</div>
<br>

> ## RF
<br>
<img src="https://user-images.githubusercontent.com/69233428/157986077-bd404058-ca5e-4a54-ad01-68713f2762ce.png">
<div align="center">
<img src="https://user-images.githubusercontent.com/69233428/157981228-7a8dd43e-2b86-45e9-9617-a57b37183c0c.gif">
   </div>

#### We can see realtime vehicle Data from ECU <br>
#### If input data doesn't come , The last Data will be continued (Above data is for example)
> ## IMU
<br>
<div align="center">
<img src="https://user-images.githubusercontent.com/69233428/157980128-d6a1ca5d-4934-4b92-9c1c-3314933801c3.png" width="500">
    </div>

#### From IMU Data show Realtime visualization
