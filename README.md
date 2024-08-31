# Welding Auto Inspection Project

## Project Overview
The Welding Auto Inspection project aims to inspect the surface of welds using a Micro Epsilon 30x2-50 laser system combined with a Basler camera for automated weld identification. The project has successfully integrated multiple modules, resulting in a fully automated inspection process.

## Key Achievements
- **High Accuracy in Weld Identification:**
  - The Vision module has achieved an accuracy rate of 95.36% in identifying welds, ensuring reliable detection and analysis.
  
- **Efficient Laser Scanning Performance:**
  - Laser scanning operates at a speed of 65 mm/s with a detailed resolution of 100 µm x 49 µm, providing precise measurements for surface inspection.

- **Advanced Software Control:**
  - The new control system allows individual robot control, enabling real-time adjustments and more accurate tracking of robot behavior.

- **Rapid 3D Visualization:**
  - The system supports 3D visualization with processing times under 1 second, typically around 0.5 seconds, ensuring swift data interpretation.

- **Comprehensive Automation:**
  - All modules (Vision, Laser, Software, Robot, and Inspection) have been fully integrated, enabling a seamless and automated inspection process for flat objects.

## Module Completion Status
| Module     | Content                                      | Status   | Note       |
|------------|----------------------------------------------|----------|------------|
| Vision     | Weld Identification                          | Done     |            |
| Laser      | Scan data                                    | Done     |            |
| Software   | Control system                               | Done     |            |
|            | Display result                               | Done     |            |
|            | View 3D                                      | Done     |            |
| Robot      | Scan data                                    | Done     |            |
| Inspection | Inspect the defect on the surface sample     | Improve  | Mr. Nhan   |
| Fix bug    | Overload memory (Python program)             | Done     |            |
|            | Post processing data laser                   | Done     |            |
|            | The robot moved to the scanning position but is misaligned by 7mm (at a position). | Consider | Mr. Quan   |

## Performance Metrics
| Part                  | Average Time (seconds) |
|-----------------------|------------------------|
| Setting system        | 0.03 seconds           |
| Weld Identification    | 0.07 seconds           |
| Moving and Laser Scan | 13.50 seconds          |
| Inspection            | 2.00 seconds           |
| View 3D               | 0.50 seconds           |
| Inspection Analysis   | ? seconds              |
| **Total Time**        | **16.57 seconds**      |

## Project Team
- **Dr. Hung Vo Tan** - Project Leader
- **Mr. Nhan Nguyen Trong** - Backend - Vision and Laser Module
- **Mr. Tuan Nguyen Dinh** - FrondEnd - Software and Control Systems Engineer
- **Mr. Quan Nguyen Dinh** - Backend - Robotics and Automation Engineer
- **Mr. Sang Nguyen Huu** - Research - Laser Control

## Technical
- Pretrained-weight: [Welding Identification and Welding Inspection](https://drive.google.com/drive/folders/1H2_BYRHt6EowTpSgy4vyFixbaU_DbkCx?usp=drive_link)
  
## Conclusion
The Welding Auto Inspection project has achieved significant milestones in developing an automated, high-accuracy, and efficient system for inspecting weld surfaces. The integration of laser scanning, real-time control software, and advanced vision systems has resulted in a robust solution that meets industrial inspection needs.

## Acknowledgement
Thanks for all the contributors.

## Contact
If you have any question, please email nguyennhan8521@gmail.com
