# 🚗 SmartScan Project

## 🌟 General Description
The main objective of the SmartScan project is to design and develop a dashboard controlled by a mobile application dedicated to automobiles. The system is capable of receiving data from the vehicle's actuators and sensors, such as engine speed, temperature, water level, and other relevant information via the CAN protocol, and transmitting it to an STM32 board for processing. The application provides a user-friendly interface allowing users to control these features directly from their desktop. Additionally, the project aims to set up a cloud-based data storage system to archive the collected data and enable remote access.

## 📁 Project Structure
The SmartScan project is divided into three main parts:

### 1. 📊 Data Acquisition
This part of the project involves collecting data from the vehicle's actuators and sensors. The data is retrieved via the CAN protocol and transmitted to an STM32 board for processing.

### 2. 🖥️ Data Visualization on the HMI
The data processed by the STM32 board is then visualized on a user-friendly Human-Machine Interface (HMI). This interface allows users to monitor and control various aspects of the vehicle directly from their desktop.

### 3. ☁️ Data Memorization
The collected data is stored in a cloud-based data storage system. This enables archiving of the data and allows remote access.

## 📷 Project Overview Image
![Project Overview](path/to/your/image.png)

Now, let's go into detail in each section.

### 1. 📊 Data Acquisition

#### Architecture
![Data Acquisition Architecture](path/to/your/image.png)

#### Implementation
For the data acquisition, we chose to work with the STM32F407 Discovery board. The solution consists of two nodes:

- **Node 1:** This node acquires data from various actuators and sensors of the vehicle. It includes components like engine speed sensors, temperature sensors, and water level sensors. The specific sensors and actuators chosen are analogous to those found in a car:
 
 ![Data Acquisition Architecture]([path/to/your/image.png](https://drive.google.com/file/d/1U7kICT-vjwQbwPRN2THw4PrEa4Zkt5ZH/view?usp=drive_link))

 1. A [DC motor] (https://www.smart-cube.biz/produit/moteur-n20-12v-1000-rpm/) analogous to the car's engine.
  2. A pedal analogous to the car's pedal.
  3. Another [DC motor] (https://www.smart-cube.biz/produit/moteur-n20-12v-1000-rpm/) analogous to the car's cooling fan.
  4. A potentiometer analogous to the car's temperature sensor.

 **Node 1 Flowchart:**
![Flowchart](path/to/your/image.png)

- **Node 2:** Similar to Node 1, this node also gathers data from other actuators and sensors in the vehicle. The specific sensors and actuators chosen are:

  ![Data Acquisition Architecture](path/to/your/image.png)

  1. A [temperature sensor] (https://www.smart-cube.biz/produit/module-dht11/) analogous to the car's coolant temperature sensor.
  2. A water [level sensor] (https://www.smart-cube.biz/produit/capteur-de-niveau-pour-les-liquides/) analogous to the car's fuel level sensor.

 **Node 2 Flowchart:**
![Flowchart](path/to/your/image.png)

Both nodes use the CAN protocol to transfer the collected data to the central node for processing. The central node, also based on the STM32F407 Discovery board, receives and processes the data to make it available for visualization and storage.

 **data transmission using can protocol Flowchart:**
![Flowchart](path/to/your/image.png)
