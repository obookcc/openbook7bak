#Arduino传感器连载之温度测量篇

>沈金鑫/南京创客空间

温度是我们经常接触到的物理量，能够被我们所直观的感受得到，例如天气凉了需要增添衣物，吃的食物太烫需要吹一吹，同时也需要对温度精确的测量，例如人类的正常体温是37.5℃，一个大气压下纯水沸腾时的温度是100℃，都需要我们去做实验来找出其中的科学。
下面我们将详细讲解几种常用的温度传感器，并利用Arduino来实现温度的测量，包括热敏电阻、LM35、DS18B20、DHT11和热电偶。

##1.热敏电阻

###1.1 热敏电阻简介
热敏电阻是电阻值随温度变化的半导体传感器，其典型特点是阻值对温度非常敏感，在不同的温度下会表现出不同的电阻值，从而根据表现的电阻值可逆推导得到其所处的环境温度值。具有灵敏度高、体积小、热容量小、响应速度快、价格低廉等优点。
按照温度系数不同，可分为正温度系数热敏电阻（PTC）、负温度系数热敏电阻（NTC）和临界负温度系数热敏电阻(CTR)。PTC随着温度升高，表现出的电阻值越大； NTC随着温度升高，表现出的电阻值越低；CTR具有负电阻突变特性，在某一温度下，电阻值随温度的增加急剧减小，具有很大的负温度系数。由于具有不同的特性，热敏电阻的用途也是不同的。PTC一般用作加热元件和过热保护；NTC一般用于温度测量和温度补偿；CTR一般用于温控报警等应用。
NTC的测温范围为－60～＋300℃，标称阻值一般在1Ω至100MΩ之间，采用精密电阻和热敏电阻组合可扩大测量温度线性范围。图1为NTC实物图，图中所示的为NTC 10D-9和NTC 5D-7。NTC表示为负温度系数的热敏电阻，10D-9和5D-7代表其型号，10D-9代表了常温（25摄氏度）阻值10欧姆，直径9毫米，5D-7代表了常温（25摄氏度）阻值5欧姆，直径7毫米。
除了图1所示的形状之外，热敏电阻制成的探头有珠状、棒杆状、片状和薄膜等，封装外壳有玻璃、镍和不锈钢管等套管结构，如图2所示。

![1](http://doask.qiniudn.com/ob7tempsensor1.jpg)
P1 NTC实物图

![2](http://doask.qiniudn.com/ob7tempsensor2.jpg)
P2 NTC的各种形式

###1.2 NTC的使用方法
NTC的测量温度和其表现出的电阻值存在一个非线性的已知的关系，那么测量出NTC的电阻值也可以计算得到其测量的温度值。NTC的电阻值与温度值的关系如下所示：

>Rt = R x e^[B x (1/T1-1/T2)]

式中， Rt 是热敏电阻在T1温度下的阻值；R是热敏电阻在T2常温下的标称阻值；B值是热敏电阻的重要参数；T1和T2指的是K度即开尔文温度，K度=273.15(绝对温度)+摄氏度。

逆向计算得到热敏电阻的温度值与电阻值的关系如下所示：

>T1=1/(ln(Rt/R) /B + 1/T2 )

电阻值的测量一般都是利用串联已知阻值的电阻并施加已知大小的电压，通过测量已知阻值的电阻上的分压值，来计算出得到被测电阻的阻值，如图3所示。设施加的激励电压为Eb，热敏电阻的阻值为Rt，串联电阻阻值为Rs，则串联电阻上的分压值为： 

Eout = Eb x Rs/(Rt+Rs)

除了串联测量法之外，还有惠斯登电桥测量法，如图4所示。设电桥的激励电压为Eb，热敏电阻的阻值为Rt，电桥电阻阻值为R1、R2和R3，则电桥输出电压为：

out = Eb x R3/(Rt+R3) - Eb x R2/(R1+R2) = Eb x [R3/(Rt+R3) - R2/(R1+R2)]

![3](http://doask.qiniudn.com/ob7tempsensor3.jpg)
P3 串联测量法  

![4](http://doask.qiniudn.com/ob7tempsensor4.jpg)
P4 电桥测量法

###1.3 使用实例
**（1）硬件连接**

此处使用串联测量法来测量来实现热敏电阻测量实验，其硬件连接图如图5所示，热敏电阻为NTC 10D-9，串联电阻的阻值为100Ω。 

![5](http://doask.qiniudn.com/ob7tempsensor5.jpg)
P5  NTC测温硬件连接图

**(2）程序设计**

程序设计的主要思路：Arduino Uno控制器通过模拟输入端口测量串联电阻上的电压值，然后通过电流相等的原理计算出热敏电阻的阻值，最后利用公式计算出温度值。

```
#include <math.h>        //包含数学库
void setup(){
  Serial.begin(9600);      //波特率设置为9600
}
void loop(){
  double Digital_Value=analogRead(0);   //读取串联电阻上的电压值（数字量）
  double Voltage_Value=(Digital_Value/1023)*5.00;//换算成模拟量的电压值
  double Rt_Value=(3.3-Voltage_Value)/Voltage_Value*100;  //计算出热敏电阻的阻值
  //计算所感知的温度并发送
Serial.println( 1/(log(Rt_Value/10)/3000 + 1/( 25 + 273.15)) - 273.15,2); 
  delay(1000);   //一秒刷新一次
}

```
##2 LM35
LM35 是美国NS（国家半导体）所生产的的模拟温度传感器，其输出的电压与摄氏温度成线性比例关系，在0℃时输出0V，温度每升高1℃，输出电压增加10mV。测温范围l ?55 ～+150?C，精度为0.75℃，室温的精度可达0.25℃。常用的TO-92封装的引脚排列如图6所示，在2℃～150℃的测温范围内的典型应用电路如图7所示。

![6](http://doask.qiniudn.com/ob7tempsensor6.jpg)
图6 TO-92封装的引脚排列 

![7](http://doask.qiniudn.com/ob7tempsensor7.jpg)
图7 2℃～150℃的典型电路图

###2.3 使用实例
**（1）硬件连接**

将LM35模拟式温度传感器的+Vs和GND分别连接至Arduino Uno控制器的+5V和GND，以给LM35提供工作电源，LM35的Vout引脚接至ArduinoUno控制器模拟输入端口A0，如图8所示。

![8](http://doask.qiniudn.com/ob7tempsensor8.jpg)
图8 LM35测温硬件连接图

**2）程序设计**

程序设计的主要思路：Arduino Uno控制器通过模拟输入端口测量LM35输出的电压值，然后通过10mV/℃的比例系数计算出温度数值。同时，在100℃的时候，LM35输出电压值为1000mV，在Arduino Uno控制器的内部参考电压范围内，所以采用1.1V内部参考电压。

```
int Digital_Value=0;
float temp_Value=0;
void setup(){
  Serial.begin(9600);      //波特率设置为9600
  //由于测温范围为0～100℃，输出电压为0～1V，采用内部1.1V参考电压
analogReference(INTERNAL);  
}
void loop(){
   Digital_Value=analogRead(A0);   //读取电压值（数字量）
   temp_Value=(float)Digital_Value/1023*110.00;//换算成摄氏温度
   Serial.print("Temperature for LM35 is: ");
   Serial.println(temp_Value,2);  //发送温度数据
   delay(1000);   //一秒刷新一次
}
```
**（3）实验演示**

实际的实验硬件连接图如图9所示，串口接收到的温度数据如图10所示。

![9](http://doask.qiniudn.com/ob7tempsensor9.jpg)
图9 实验硬件连接图

![10](http://doask.qiniudn.com/ob7tempsensor10.jpg)
图10 串口接收的温度数据

##3 DS18B20
###3.1 DS18B20简介
DS18B20是美国DALLAS半导体公司的数字化单总线智能温度传感器，与传统的热敏电阻相比，它能够直接读出被测温度，并且可根据实际要求通过简单的编程实现9～12位的数字值读数方式。从DS18B20读出信息或写入信息仅需要一根线(单总线)读写，总线本身也可以向所挂接的设备供电，而无需额外电源。

DS18B20的性能特点如下:

>(1)	单线接口方式实现双向通讯； 

>(2)	供电电压范围：+3.0V～+5.5V，可用数据线供电；

>(3)	测温范围：-55～+125℃，固有测温分辨率为0.5℃；

>(4)	通过编程可实现9～12位的数字读数方式；

>(5)	支持多点的组网功能，多个DS18B20可以并联在唯一的单总线上，实现多点测温。

DS18B20的外形及管脚排列如图11所示，DS18B20引脚定义：(1)DQ为数字信号输入/输出端；(2)GND为电源地；(3)VDD为外接供电电源输入端（在寄生电源接线方式时接地）。

![11]()
图11 DS18B20封装图

###3.2 DS18B20编程与库的使用
Arduino要实现对DS18B20的操作，需要OneWire和Dallas Temperature Control两个库文件，下载地址分别为：http://playground.arduino.cc/Learning/OneWire和https://github.com/milesburton/Arduino-Temperature-Control-Library。Dallas Temperature Control函数库是基于OneWire函数库进行开发的，更便于使用，下面讲解一下主要函数的功能和用法。

>(1)	void begin(void)：初始化，无输入参数，无返回参数。

>(2)	getDeviceCount(void)：获取单总线上所连接器件的总数，无输入参数，返回参数为器件数目。

>(3)	validAddress(uint8_t*)：验证指定地址的器件是否存在，输入参数为器件地址，返回参数为布尔型。

>(4)	getAddress(uint8_t*， const uint8_t)：验证的器件的地址与索引值是否匹配，输入参数为器件地址和索引值，返回参数为布尔型。

>(5)	getResolution(uint8_t*)：获取指定器件的精度，输入参数为器件地址，返回参数为精度位数。

>(6)	setResolution(uint8_t*，uint8_t)：设置器件的精度，输入参数为器件地址和精度位数，无返回参数。精度位数有9，10，11和12可供选择。

>(7)	requestTemperatures(void)：向单总线上所有器件发送温度转换的请求，无输入参数，无返回参数。

>(8)	requestTemperaturesByAddress(uint8_t*)：向单总线上指定地址的器件发送温度转换的请求，输入参数为器件地址，无返回参数。   

>(9)	requestTemperaturesByIndex(uint8_t) ：向单总线上指定索引值的器件发送温度转换的请求，输入参数为器件索引值，无返回参数。

>(10)	getTempC(uint8_t*)：通过器件地址获取摄氏温度，输入参数为器件地址，返回参数为摄氏温度。

>(11)	getTempF(uint8_t*)：通过器件地址获取华氏温度，输入参数为器件地址，返回参数为华氏温度。

>(12)	getTempCByIndex(uint8_t)：通过索引值来获取摄氏温度，输入参数为器件索引值，返回参数为摄氏温度。

>(13)	getTempFByIndex(uint8_t)：通过器件索引值来获取华氏温度，输入参数为器件索引值，返回参数为华氏温度。

###3.3 使用实例

由于单总线上可以连接多个DS18B20，而不多占用Arduino控制器的IO口，从而很容易地就可以实现多点测温，下面分别两个使用实例来说明DS18B20与Arduino的用法。

####3.3.1 一路温度测量

**（1）硬件连接**

将DS18B20温度传感器的VCC和GND分别连接至Arduino Uno控制器的+5V和GND，以给DS18B20提供电源，DS18B20的DQ引脚接至ArduinoUno控制器数字引脚D2，且并联4.7kΩ的上拉电阻，如图12所示。

![12](http://doask.qiniudn.com/ob7tempsensor12.jpg)
图12 一路温度测量硬件连接图

（2）程序设计
程序设计的主要思路：Arduino Uno控制器通过DallasTemperature函数库实现单总线的启动、发送测量温度的请求、读取0号传感器温度，最后通过串口发送出去。

```
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2    //定义单总线连接的端口
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");
  sensors.begin();    //启动单总线
}

void loop(void)
{ 
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();    //发送温度测量请求命令
  Serial.println("DONE");
  
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.print(sensors.getTempCByIndex(0));    //获取0号传感器温度数据并发送
  Serial.println("℃");  
  delay(1000);   //一秒刷新一次
}
```

####3.3.2 多路温度测量

**（1）硬件连接**

将两个DS18B20温度传感器的VCC和GND分别连接至Arduino Uno控制器的+5V和GND，以给两个DS18B20提供电源，两个DS18B20的DQ引脚接至ArduinoUno控制器数字引脚D2，且并联4.7kΩ的上拉电阻，如图13所示。

![13](http://doask.qiniudn.com/ob7tempsensor13.jpg)
图13 多路温度测量硬件连接图

**（2）程序设计**

程序设计的主要思路：Arduino Uno控制器通过DallasTemperature函数库实现单总线的启动、发送测量温度的请求、读取0号传感器温度并串口发送出去，读取1号传感器温度并串口发送出去。

```
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2    //定义单总线连接端口
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");
  sensors.begin();   //启动单总线
}

void loop(void)
{ 
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();   //发送温度测量请求命令
  Serial.println("DONE");  
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(0));  //获取0号传感器温度数据并发送
Serial.print("Temperature for the device 2 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(1));  //获取1号传感器温度数据并发送
}
```

####3.3.2 实验演示

实际的单路实验硬件连接图如图14所示，单路和两路实验中串口接收到的温度数据分别如图15和16所示。

![14](http://doask.qiniudn.com/ob7tempsensor4.jpg)
图14 单路实验硬件连接图

![15](http://doask.qiniudn.com/ob7tempsensor15.jpg)
图15 单路串口接收的温度数据

![16](http://doask.qiniudn.com/ob7tempsensor16.jpg)
图16 两路串口接收的温度数据

##4 DHT11
###4.1  DHT11简介
DHT11是一款含有已校准系数的数字信号输出的温湿度复合传感器，采用专用的数字模块采集技术和温湿度传感技术，具有极高的可靠性与卓越的长期稳定性，内部包含一个电阻式感湿元件和一个NTC测温元件。DHT11传感器都经过实验室校准，校准系数以程序的形式储存在OTP内存中，传感器内部在检测信号的处理过程中要调用这些校准系数，采用单线制串行接口，使系统集成变得简易快捷。超小的体积、极低的功耗，信号传输距离可达20米以上。DHT11数字温湿度传感器实物图如17所示。

![17}()
图17 DHT11温湿度传感器

DHT11的引脚说明如表1所示，供电电压为3.3～5V，测量范围为湿度20～90%RH， 温度0～50℃，测量精度为湿度±5%RH，温度±2℃，测量分辨率为湿度1%RH，温度1℃。

![f1](http://doask.qiniudn.com/ob7tempsensorf1.jpg)

###4.2  DH11编程与库的使用
DHT11的Arduino库文件下载地址：https://github.com/markruys/arduino-DHT。DHT11库文件有如下几个函数：dht.setup(int Pin)、dht.getHumidity()、dht.getTemperature()。

>dht.setup(int Pin)：设置DHT11总线的连接管脚号，输入参数为所连接的管脚号，无返回参数。

>dht.getHumidity()：获取DHT11的湿度值，无输入参数，返回值为湿度值，double类型。

>dht.getTemperature()：获取DHT11的温度值，无输入参数，返回值为温度值，double类型。

###4.3 使用实例
下面以DHT11模块实现温湿度的测量，并且通过串口输出。
（1）硬件连接
将DHT11温湿度传感器的VCC、GND分别连接至Arduino Uno控制器的+5V、GND，以给DHT11提供电源，DHT11模块的DOUT引脚接至ArduinoUno控制器数字引脚D2，且并联5kΩ的上拉电阻，DHT11模块的NC引脚也连接至GND，如图18所示。

![18](http://doask.qiniudn.com/ob7tempsensor18.jpg)
图18 DHT11温湿度测量硬件连接图

（2）程序设计
程序设计的主要思路：Arduino Uno控制器通过DHT11函数库获取湿度和温度数据，并通过串口发送出去。
#include "DHT.h"

DHT dht;

void setup()
{
  Serial.begin(9600);
  dht.setup(2); // data pin 2
  delay(1000);
}

void loop()
{
  float temperature = dht.getTemperature();
  float humidity = dht.getHumidity();
  
  Serial.print("temperature is ");
  Serial.print(temperature, 1);  
  Serial.println(" C");
  Serial.print("humidity is ");
  Serial.print(humidity, 1);
  Serial.println("%");
  delay(3000);
}
（3）实验演示
实际的实验硬件连接图如图19所示，实验中串口接收到的温湿度数据如图20所示。

![19](http://doask.qiniudn.com/ob7tempsensor19.jpg)
图19 实验硬件连接图

![20](http://doask.qiniudn.com/ob7tempsensor20.jpg)
图20 串口接收的温湿度数据

##5  热电偶

###5.1  热电偶和MAX6675简介

将两种不同材料的导体或半导体A和B焊接起来，构成一个闭合回路，当导体A和B的两个连接点1和2之间存在温差时，两者之间便产生电动势，因而在回路中形成一个回路电流。这种现象称为热电效应，而这种电动势称为热电势。热电效应原理图如图21所示。

![21](http://doask.qiniudn.com/ob7tempsensor1.jpg)
图21 热电效应原理图

热电偶就是利用热电原理进行温度测量的，其中，直接用作测量介质温度的一端叫做工作端（也称为测量端），另一端叫做冷端（也称为补偿端）。实际上是一种能量转换器，它将热能转换为电能，用所产生的热电势测量温度。
常用的K型热电偶实物如图22所示，可以直接测量各种生产中从0℃到1300℃范围的液体蒸汽和气体介质以及固体的表面温度。具有线性度好，热电动势较大，灵敏度高，稳定性和均匀性较好，抗氧化性能强，价格便宜等优点。 

![22](http://doask.qiniudn.com/ob7tempsensor1.jpg)
图22 K型热电偶实物图

根据热电偶测温原理，K型热电偶的输出热电势不仅与测量端的温度有关，而且与冷端的温度有关，需要温度补偿电路（如图23为补偿示意图），同时热电偶的电压与温度之间具有非线性，
MAX6675模块可以对K型热电偶进行信号放大、冷端补偿和非线性校正。
MAX6675带有简单的3位串行SPI接口；可将温度信号转换成12位数字量，
温度分辨率达0.25℃；内含热电偶断线检测电路。冷端补偿的温度范围-20℃～80℃，可以测量0℃～1023.75℃的温度，基本符合工业上温度测量的需要。

###5.2  MAX6675编程与库的使用

MAX6675的Arduino库文件下载地址：https://github.com/aguegu/ardulibs/tree/master/max6675。MAX6675库文件有如下几个函数：getCelsius()、getFahrenheit()、getKelvin()和setOffset(int offset)。
?
>getCelsius()：获取摄氏温度，无输入参数，返回值为摄氏温度，float类型。
?	
>getFahrenheit()：获取华氏温度，无输入参数，返回值为华氏温度，float类型。
?	
>getKelvin()：获取开尔文温度，无输入参数，返回值为开尔文温度，float类型。
?	
>setOffset(int offset)：设置温度偏移，输入参数为偏移值，int类型，最小单位为0.25℃，无返回值。

###5.3  使用实例

下面以K型热电偶与MAX6675模块实现高温的测量，并且通过串口输出。

**（1）硬件连接**

将MAX6675模块的VCC和GND分别连接至Arduino Uno控制器的+5V和GND，以给MAX6675提供电源，MAX6675模块的信号引脚SO、CS和CSK连接至数字引脚5、6、7，K型热电偶的正负极分别连接至MAX6675模块的T+和T-，如图24所示。

![24](http://doask.qiniudn.com/ob7tempsensor24.jpg)
图24 热电偶测温硬件连接图

**（2）程序设计**

程序设计的主要思路：Arduino Uno控制器通过MAX6675函数库获取热电偶所测量的温度值，完成了热电偶输出电压的信号放大、冷端补偿和非线性化处理，最终通过串口输出。

```
#include "Max6675.h"
Max6675 ts(5, 6, 7);             //依次定义SO、CS、CSK所连接的引脚号
void setup(){
ts.setOffset(0);               //设置温度偏移量
Serial.begin(9600);
}

void loop(){
  Serial.print("temperature is ");
  Serial.println(ts.getCelsius(), 2);   //获取摄氏温度，并通过串口发送
 delay(1000);                     //一秒刷新一次
}
```

**（3）实验演示**

实际的实验硬件连接图如图25所示，实验中串口接收到的温度数据如图26所示。

![25](http://doask.qiniudn.com/ob7tempsensor25.jpg)
图25 实验硬件连接图

![26](http://doask.qiniudn.com/ob7tempsensor26.jpg)
图26 串口接收的温度数据

##6.总结

本文介绍了温度测量的几种常用传感器，从测温原理、器件特性、在Arduino中的编程与使用等方面做了详细的介绍。总结本文，有以下几点：

>1、NTC热敏电阻价格低廉，但是想要得到很高的测量精度，需要做很多优化工作，难度较大。

>2、LM35直接输出模拟电压，使用较为方便，精度较高，适合用于热电偶冷端补偿中的环境温度测量。

>3、DS18B20是单总线数字温度传感器，性价比较高，测量精度较高，同时可以单个总线挂多个传感器。

>4、DHT11是温湿度传感器，单总线，不占用过多的I/O口，而且可以同时输出湿度数据，适合同时需要温湿度数据的场合应用。

>5、热电偶和MAX6675配合使用，适合高温测量，省去了热电偶的冷端补偿、线性化和模数转换等工作，使用较方面，精度较高，对其数据进行二次拟合标定，可以得到更高的测量精度。

最后对比一下热电偶+MAX6675模块和DS18B20的响应速度，如图27所示是基于Arduino与LabVIEW的实验平台采集到热电偶在放进热水中的数据变化情况，从图中可以看出，最高温度约为60℃，热电偶的响应曲线较为平直，上升速度较快。如图28所示为基于Arduino与LabVIEW的实验平台采集到DS18B20对于冷热变化的响应曲线图，从图中可以看出最高温度超过80℃，DS18B20响应曲线较为平缓，随着温差的缩小，温度响应速度越发放缓。

![27](http://doask.qiniudn.com/ob7tempsensor27.jpg)
图27 热电偶温度变化响应曲线图

![28](http://doask.qiniudn.com/ob7tempsensor28.jpg)
图28 DS18B20温度变化响应曲线图

