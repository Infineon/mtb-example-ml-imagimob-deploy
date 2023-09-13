# Machine learning: Imagimob model deployment

This code example demonstrates how to deploy an Imagimob-generated machine learning model. It comes pre-configured with a model generated from the Human Activity Detection starter project in Imagimob Studio. The code example collects accelerometer data from an IMU, which is then sent to the machine learning model to detect specific motions (sitting, standing, walking, running, or jumping). It uses the model.c/h files generated from within Imagimob Studio directly. New models based on the Human Activity Detection project can be dropped into the project as-is.

This example does not use the ModusToolbox&trade; Machine Learning Configurator or any device-specific optimizations.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-ml-imagimob-deploy)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzg0NzIiLCJTcGVjIE51bWJlciI6IjAwMi0zODQ3MiIsIkRvYyBUaXRsZSI6Ik1hY2hpbmUgbGVhcm5pbmc6IEltYWdpbW9iIG1vZGVsIGRlcGxveW1lbnQiLCJyaWQiOiJucnNoIiwiRG9jIHZlcnNpb24iOiIxLjAuMCIsIkRvYyBMYW5ndWFnZSI6IkVuZ2xpc2giLCJEb2MgRGl2aXNpb24iOiJNQ0QiLCJEb2MgQlUiOiJJQ1ciLCJEb2MgRmFtaWx5IjoiUFNPQyJ9)



## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.1 or later (tested with v3.1)
- Programming language: C
- Associated parts: All [PSoC&trade; 6 MCU](https://www.infineon.com/PSoC6) parts
- PSoC&trade; 6 board support package (BSP) minimum required version: 4.0.0


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.16 (`ARM`)
- IAR C/C++ Compiler v9.30.1 (`IAR`)


## Supported kits (make variable 'TARGET')

- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CY8CKIT-062S2-43012) (`CY8CKIT-062S2-43012`) – Default value of `TARGET`
- [PSoC&trade; 6 Bluetooth&reg; LE Pioneer Kit](https://www.infineon.com/CY8CKIT-062-BLE) (`CY8CKIT-062-BLE`)
- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CY8CKIT-062-WIFI-BT) (`CY8CKIT-062-WIFI-BT`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CYW9P62S1-43438EVB-01) (`CYW9P62S1-43438EVB-01`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CYW9P62S1-43012EVB-01) (`CYW9P62S1-43012EVB-01`)
- [PSoC&trade; 62S2 Evaluation Kit](https://www.infineon.com/CY8CEVAL-062S2) (`CY8CEVAL-062S2`, `CY8CEVAL-062S2-LAI-4373M2`, `CY8CEVAL-062S2-MUR-43439M2`, `CY8CEVAL-062S2-LAI-43439M2`, `CY8CEVAL-062S2-MUR-4373EM2`, `CY8CEVAL-062S2-MUR-4373M2`)
- [PSoC&trade; 62S4 Pioneer Kit](https://www.infineon.com/CY8CKIT-062S4) (`CY8CKIT-062S4`)
- [PSoC&trade; 64 "Secure Boot" Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CY8CKIT-064B0S2-4343W) (`CY8CKIT-064B0S2-4343W`)
- [PSoC&trade; 64 Standard Secure - AWS Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CY8CKIT-064S0S2-4343W) (`CY8CKIT-064S0S2-4343W`)

## Hardware setup

Connect an Arduino shield ([CY8CKIT-028-EPD](https://www.infineon.com/CY8CKIT-028-EPD), [CY8CKIT-028-SENSE](https://www.infineon.com/CY8CKIT-028-SENSE), or [CY8CKIT-028-TFT](https://www.infineon.com/CY8CKIT-028-TFT)) to the baseboard's Arduino header.

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.


> **Note:** The PSoC&trade; 6 Bluetooth&reg; LE Pioneer Kit (CY8CKIT-062-BLE) and the PSoC&trade; 6 Wi-Fi Bluetooth&reg; Pioneer Kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. ModusToolbox&trade; requires KitProg3. Before using this code example, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the [Firmware Loader](https://github.com/Infineon/Firmware-loader) GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".



## Software setup

Install [Imagimob Studio](https://developer.imagimob.com/) if not already installed.

Install a terminal emulator if you don't have one. Instructions in this document use [Tera Term](https://ttssh2.osdn.jp/index.html.en).


## Using the code example

### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool.

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*).

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target).

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**.

   > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you.

   b.	Select this code example from the list by enabling its check box.

   > **Note:** You can narrow the list of displayed examples by typing in the filter box.

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**.

   d. Click **Create** to complete the application creation process.

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[mtb-example-ml-imagimob-deploy](https://github.com/Infineon/mtb-example-ml-imagimob-deploy)" application with the desired name "ImagimobModelDeploy" configured for the *CY8CKIT-062S2-43012* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id CY8CKIT-062S2-43012 --app-id mtb-example-ml-imagimob-deploy --user-app-name ImagimobModelDeploy --target-dir "C:/mtb_projects"
   ```


The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>



### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/MTBVSCodeUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Keil µVision</b></summary>

Double-click the generated *{project-name}.cprj* file to launch the Keil µVision IDE.

For more details, see the [Keil µVision for ModusToolbox&trade; user guide](https://www.infineon.com/MTBuVisionUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_uvision_user_guide.pdf*).

</details>


<details><summary><b>IAR Embedded Workbench</b></summary>

Open IAR Embedded Workbench manually, and create a new project. Then select the generated *{project-name}.ipcf* file located in the project directory.

For more details, see the [IAR Embedded Workbench for ModusToolbox&trade; user guide](https://www.infineon.com/MTBIARUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_iar_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Provisioning steps (Only required for Secured kits)


If you are using one of the PSoC&trade; 64 Secured kits, then the PSoC&trade; 64 device must be provisioned with keys and policies before being programmed. Do the following instructions depending on your kit.

> **Note:** The KitProg3 must be in DAPLink mode for performing this section. Press the **Mode** button on the kit until the Status LED blinks fast. In addition, ensure that the J26 jumper is open.

<details><summary><b> If using a PSoC&trade; 64 "Standard Secure" kit (`CY8CKIT-064S0S2-4343W`) </b></summary>


   1. Navigate to the *_<mtb_shared>/trusted-firmware-m/< release tag >/security/_* folder in the modus shell.

   2. Run the following command.

      ```
      cysecuretools --target CY8CKIT-064S0S2-4343W init
      ```

   3. Generate new keys to sign the image. Run the following command:

      ```
      cysecuretools --target CY8CKIT-064S0S2-4343W -p policy/policy_multi_CM0_CM4_tfm.json create-keys
      ```

   4. Provision the device. Run the following command:

      ```
      cysecuretools --target CY8CKIT-064S0S2-4343W -p policy/policy_multi_CM0_CM4_tfm.json provision-device
      ```

      > **Note:** If your device is already provisioned, then use the following command:

      ```
      cysecuretools --target CY8CKIT-064S0S2-4343W -p policy/policy_multi_CM0_CM4_tfm.json re-provision-device
      ```

</details>

<details><summary><b>If using a PSoC&trade; 64 "Secure Boot" kit (`CY8CKIT-064B0S2-4343W`)</b></summary>


   If you are using the [CY8CKIT-064B0S2-4343W](https://www.infineon.com/CY8CKIT-064B0S2-4343W) kit, do the following steps. See ["Secure Boot" SDK user guide](https://www.infineon.com/dgdlac/Infineon-PSoC_64_Secure_MCU_Secure_Boot_SDK_User_Guide-Software-v07_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f8c361a7666) to know how to provision other PSoC&trade; 64 Secured kits.

   1. Navigate to the *_<Application_Name>_* folder in the modus shell.

   2. Run the following command:

      ```
      cysecuretools --target CY8CKIT-064B0S2-4343W init
      ```

   3. Generate new keys to sign the image. Run the following command:

      ```
      cysecuretools --target CY8CKIT-064B0S2-4343W -p policy/policy_single_CM0_CM4_swap.json create-keys
      ```

   4. Provision the device. Run the following command:

      ```
      cysecuretools --target CY8CKIT-064B0S2-4343W -p policy/policy_single_CM0_CM4_swap.json provision-device
      ```

      > **Note:** If your device is provisioned earlier, then use the following command:

      ```
      cysecuretools --target CY8CKIT-064B0S2-4343W -p policy/policy_single_CM0_CM4_swap.json re-provision-device
      ```

</details>


## Operation

If using a PSoC&trade; 64 "Secure" MCU kit, follow the steps mentioned in the [Provisioning steps](#provisioning-steps-only-required-for-secured-kits).

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>


   <details><summary><b>In other IDEs</b></summary>

   Follow the instructions in your preferred IDE.
   </details>


   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, the application starts automatically. Confirm that "Imagimob Machine Learning Deploy model.c Example" displays on the UART terminal.

    **Figure 1. Terminal output on program startup**

    ![](images/terminal.jpg)

5. Confirm that the kit LED is illuminated and pulsing. This indicates that the example is running. It will blink each time an inference operation is about to begin.

6. Perform various activities (sitting, standing, walking, running, jumping) and observe that the model detects and reports the correct activity. For proper detection, the sensor on the board must be oriented in the same general manner in which it is trained. The orientation for sitting and standing are shown in Figure 2 and Figure 3 respectively. For walking, running, and jumping the board is held the same as standing with the normal arm movements associated with the respective activity.

      **Figure 2. Sitting: KitProg USB facing forward, shield toward the ground**

      ![](images/sitting.jpg)

      **Figure 3. Standing: KitProg USB facing forward, shield toward the body**

      ![](images/standing.jpg)

7. The model.c/h files can be updated to use other models generated by the Imagimob Studio. The current model and the sensor data collection are based on the [Human Activity Recognition Starter Project](https://developer.imagimob.com/getting-started/starter-project). [Machine learning: Imagimob data collection](https://github.com/Infineon/mtb-example-imagimob-data-collection) can be used to capture additional sensor data for training the model or to create a new dataset for a different type of model. Open Imagimob Studio to make changes and experiment further.


## Debugging


You can debug the example to step through the code.


<details><summary><b>In Eclipse IDE</b></summary>

Use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

> **Note:** **(Only while debugging)** On the CM4 CPU, some code in `main()` may execute before the debugger halts at the beginning of `main()`. This means that some code executes twice – once before the debugger stops execution, and again after the debugger resets the program counter to the beginning of `main()`. See [KBA231071](https://community.infineon.com/docs/DOC-21143) to learn about this and for the workaround.

</details>


<details><summary><b>In other IDEs</b></summary>

Follow the instructions in your preferred IDE.
</details>



## Design and implementation

In this example, the firmware reads the data from a motion sensor (BMX160 or BMI160) to detect human activity.

The data consists of the 3-axis accelerometer data obtained from the motion sensor. A timer is configured to interrupt at 50 Hz to sample the motion sensor. The interrupt handler reads all the data from the sensor via I2C or SPI and queues the values up for the machine learning model's inference engine. Once at least 50 frames of data have been captured, the inference engine is run to determine the activity that is occurring. Each possible activity is assigned a confidence score. The confidence score output from the inference engine is then printed on the UART terminal for review.

### Configuration

This code example is designed to work with one of the Arduino Shields produced by Infineon that includes a motion sensor. To select the shield that is currently being used, modify the Makefile to change the define that is being specified. By default, the example uses the CY8CKIT-028-SENSE shield v1. The valid options are:
   * EPD_SHIELD: For the CY8CKIT-028-EPD with the BMI-160 sensor
   * SENSE_SHIELDv1: For the CY8CKIT-028-SENSE with the BMX-160 sensor
   * SENSE_SHIELDv2: For the CY8CKIT-028-SENSE with the BMI-160 sensor
   * TFT_SHIELD: For the CY8CKIT-028-TFT with the BMI-160 sensor

### Activity classification model

The model uses a 50-frame sliding window for evaluating data over a period of 1 second.

The model is a standard convolutional neural network (CNN) model consisting of five convolutional groups. These groups are divided into three groups of convolutional blocks (one or two layers), a batch normalization layer, a rectified linear unit (ReLU) activation layer, and a max pooling layer.

   **Figure 4. Human Activity Model layers**

   ![](images/layers.jpg)

The convolutional layers act as feature extractors and provide abstract representations of the input sensor data in feature maps. They capture short-term dependencies (spatial relationships) of the data. In the CNN, features are extracted and then used as inputs to later feature extractors, finally a softmax activation is used for final classification.

### Model generation

This code example ships with the model.c/h files produced by the Human Activity Detection started project in Imagimob Studio. Imagimob Studio can be used to capture new data as well as review, modify, or generate new models for evaluation. To use a new model, copy the Edge model.c/h files produced by the tool into this project.

### Resources and settings

**Table 1. Application resources**

 Resource  |  Alias/object     |    Purpose
 :-------- | :-------------    | :------------
 GPIO (HAL)    | CYBSP_USER_LED     | User LED
 UART (HAL)|cy_retarget_io_uart_obj| UART HAL object used by Retarget-IO for the Debug UART port
 Timer (HAL) | imu_read_timer  | Timer HAL object used to periodically read from the IMU
 I2C (HAL) | i2c_obj           | I2C HAL object used to communicate with the IMU sensor (used for the [CY8CKIT-028-EPD](https://www.infineon.com/CY8CKIT-028-EPD) or [CY8CKIT-028-TFT](https://www.infineon.com/CY8CKIT-028-TFT) shields)
 SPI (HAL) | spi_obj           | SPI HAL object used to communicate with the IMU sensor (used for the [CY8CKIT-028-SENSE](https://www.infineon.com/CY8CKIT-028-SENSE) shield)

<br>



## Related resources


Resources  | Links
-----------|----------------------------------
Application notes  | [AN228571](https://www.infineon.com/AN228571) – Getting started with PSoC&trade; 6 MCU on ModusToolbox&trade; <br>  [AN215656](https://www.infineon.com/AN215656) – PSoC&trade; 6 MCU: Dual-CPU system design
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [PSoC&trade; 6 MCU datasheets](https://documentation.infineon.com/html/psoc6/bnm1651211483724.html) <br> [PSoC&trade; 6 technical reference manuals](https://documentation.infineon.com/html/psoc6/zrs1651212645947.html)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board).
Libraries on GitHub  | [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) – PSoC&trade; 6 Peripheral Driver Library (PDL)  <br> [mtb-hal-cat1](https://github.com/Infineon/mtb-hal-cat1) – Hardware Abstraction Layer (HAL) library <br> [retarget-io](https://github.com/Infineon/retarget-io) – Utility library to retarget STDIO messages to a UART port
Middleware on GitHub  | [psoc6-middleware](https://github.com/Infineon/modustoolbox-software#psoc-6-middleware-libraries) – Links to all PSoC&trade; 6 MCU middleware
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSoC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development.

<br>

## Other resources


Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.



## Document history


Document title: *CE238472* - *Machine learning: Imagimob model deployment*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example
<br>



All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.


---------------------------------------------------------

© Cypress Semiconductor Corporation, 2023. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.
