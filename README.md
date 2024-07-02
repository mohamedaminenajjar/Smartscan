 🚗 SmartScan Project

## 🌟 General Description
The main objective of the SmartScan project is to design and develop a dashboard controlled by a mobile application dedicated to automobiles. The system is capable of receiving data from the vehicle's actuators and sensors, such as engine speed, temperature, water level, and other relevant information via the CAN protocol, and transmitting it to an STM32 board for processing. The application provides a user-friendly interface allowing users to control these features directly from their desktop. Additionally, the project aims to set up a cloud-based data storage system to archive the collected data and enable remote access.

### 🎥 Project Demo


https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/15054ed6-27c9-43cc-a34a-e9932c1f3cfa


## 📁 Project Structure
The SmartScan project is divided into three main parts:

### 1. 📊 Data Acquisition
This part of the project involves collecting data from the vehicle's actuators and sensors. The data is retrieved via the CAN protocol and transmitted to an STM32 board for processing.

### 2. 🖥️ Data Visualization on the HMI
The data processed by the STM32 board is then visualized on a user-friendly Human-Machine Interface (HMI). This interface allows users to monitor and control various aspects of the vehicle directly from their desktop.

### 3. ☁️ Data Memorization
The collected data is stored in a cloud-based data storage system. This enables archiving of the data and allows remote access.

## 📷 Project Overview Image


![Capture d'écran 2024-07-01 234232](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/1f6db4e9-ae3b-4eb1-8abd-d40304ab9aa6)


Now, let's go into detail in each section.

### 1. 📊 Data Acquisition

#### Architecture

For the data acquisition, we chose to work with the STM32F407 Discovery board and the CAN protocol to transmit the data from nodes 1 and 2 to central node.

![Capture d'écran 2024-07-02 003502](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/368e4252-6b97-47d8-94f8-5e6ee869fb9a)


- **Node 1:** This node acquires data from various actuators and sensors of the vehicle. It includes components like engine speed sensors, temperature sensors, and water level sensors. The specific sensors and actuators chosen are analogous to those found in a car:
 
![capteu_actionneur1](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/8e7cf161-87c7-4ec7-8a2c-2438844d89f2)

 1. A [DC motor](https://www.smart-cube.biz/produit/moteur-n20-12v-1000-rpm/) analogous to the car's engine.
  2. A pedal analogous to the car's pedal.
  3. Another [DC motor](https://www.smart-cube.biz/produit/moteur-n20-12v-1000-rpm/) analogous to the car's cooling fan.
  4. A potentiometer analogous to the car's temperature sensor.

 **Node 1 Flowchart:**

![organigramme_node1](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/650a8326-232b-4662-9484-68c3bcc75bec)


- **Node 2:** Similar to Node 1, this node also gathers data from other actuators and sensors in the vehicle. The specific sensors and actuators chosen are:


![capteur_actionneur2](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/ad785be6-c150-48d7-88a9-7a836ad83a8b)


  1. A [temperature sensor](https://www.smart-cube.biz/produit/module-dht11/) analogous to the car's coolant temperature sensor.
  2. A water [level sensor](https://www.smart-cube.biz/produit/capteur-de-niveau-pour-les-liquides/) analogous to the car's fuel level sensor.

 **Node 2 Flowchart:**

![organigramme_node2](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/8267ff65-dfdd-4734-831c-660ceb89e0ea)


Both nodes use the CAN protocol to transfer the collected data to the central node for processing. The central node, also based on the STM32F407 Discovery board, receives and processes the data to make it available for visualization and storage.

 **data transmission using can protocol Flowchart:**

![organigramme_can](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/a624d08e-cd64-4cca-95be-2bd2d008a2cd)

### 2. 🖥️ Data Visualization on the HMI
In this part of the project, we transmit the data acquired by the central node to the Human-Machine Interface (HMI) to allow users to visualize real-time data. We utilized [Qt Creator](https://www.qt.io/product/development-tools) to develop this HMI.

#### Transmission Protocol Issue
We initially chose the UART protocol for transmitting data from the central node to the HMI. However, we encountered an issue as the PC does not support UART directly. To resolve this, we employed an [FTDI USB vers TTL](https://souilah-electronique.tn/modules-et-capteurs/120-module-adaptateur-serie-ft232rl-usb-vers-ttl-5v-33v-pour-arduino.html), which converts UART signals to USB protocol for communication.

#### HMI Pages
Our HMI consists of two main pages:

1. **Identification Page**

![Capture d'écran 2024-07-02 012413](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/982a487f-bc91-4fd6-bc48-af72e248b50d)

   This page serves as the login or identification interface where users authenticate themselves to access the system.

2. **Real-Time Data Visualization Page**

![Capture d'écran 2024-05-24 033750](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/67ad25ca-2d79-44b8-a70e-11d025704f34)

   This page displays real-time data acquired from the vehicle's sensors and actuators. Users can monitor various parameters such as engine speed, temperature, and water level directly on this interface.

By integrating these components, we provide a user-friendly interface that enhances the monitoring and control capabilities of the SmartScan system.

### 3. ☁️ Data Memorization
This part of the project focuses on storing data permanently to facilitate retrieval. We chose to use the [ThingSpeak cloud](https://thingspeak.com) platform where data from the central node is transferred using the WiFi protocol.

#### Protocol Issue
Initially, we planned to use WiFi for transmitting data from the central node to ThingSpeak. However, the STM32F407VGT6 microcontroller does not support WiFi directly. To address this issue, we integrated the [ESP8266 WiFi module](https://www.didactico.tn/produit/module-wifi-esp-01-esp8266-2/). This module facilitates the conversion of UART signals to WiFi protocol, enabling seamless data transmission to the cloud.

#### ThingSpeak Cloud Integration
Data transmitted from the central node is securely stored on the ThingSpeak cloud platform. This setup ensures that collected data is archived and readily accessible for analysis and monitoring purposes.

By leveraging the ESP8266 module and ThingSpeak integration, we ensure robust data storage capabilities for the SmartScan system, enhancing its functionality and usability.

![Capture d'écran 2024-07-02 014456](https://github.com/mohamedaminenajjar/Smartscan/assets/152156763/82b7b761-f987-4152-b318-abadee542d0f)

### 📄 Project Presentation
For a detailed presentation of the project, please refer to the PowerPoint file:

[Project Presentation (.pptx)](link_to_your_presentation.pptx)

## Contact 📧

For any questions or inquiries, please contact:
- Email: [mohamedamine.najjar@isimg.tn](https://mail.google.com/mail/u/0/?fs=1&tf=cm&source=mailto&to=mohamedamine.najjar@isimg.tn)
- LinkedIn: [Mohamed Amine Najjar](https://www.linkedin.com/in/mohamed-amine-najjar-2808a726b/)

---

Happy coding! 😊
