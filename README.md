# [Spring 2025] Autonomous Mobile Robot

![Last Commit](https://img.shields.io/github/last-commit/Choroning/25Spring_Autonomous-Mobile-Robot)
![Languages](https://img.shields.io/github/languages/top/Choroning/25Spring_Autonomous-Mobile-Robot)

This repository archives bilingual lecture notes and coursework for the Autonomous Mobile Robot course.

*Author: Cheolwon Park (Korea University Sejong, CSE) - Year 2 (Sophomore) as of 2025*
<br><br>

## 📑 Table of Contents

- [About This Repository](#about-this-repository)
- [Course Information](#course-information)
- [Prerequisites](#prerequisites)
- [Repository Structure](#repository-structure)
- [License](#license)

---


<br><a name="about-this-repository"></a>
## 📝 About This Repository

- `Lecture-Notes/` holds a lecture note for each week (`Week01`, `Week02`, and so on) that covers that week's lecture materials, and Weeks 5-7 also hold the ROS node example code used in the lectures.
- Every lecture note comes as an English (`.md`) and Korean (`.ko.md`) pair.

> **🤖 AI Agent Usage**
> [Claude Code](https://claude.ai/download) was used to organize and translate the notes and to restructure the repository.

<br><a name="course-information"></a>
## 📚 Course Information

- **Semester:** Spring 2025 (March - June)
- **Affiliation:** Korea University Sejong

| Course&nbsp;Code| Course            | Type          | Instructor      | Department                              |
|:----------:|:------------------|:-------------:|:---------------:|:----------------------------------------|
|`DAMO425-00`|AUTONOMOUS MOBILE ROBOT|Major Elective|Prof. Youngeun&nbsp;Song|Department of Autonomous Mobility|

### Course Overview

A major elective course that teaches, through theory and practice, the structure of mobile robot navigation systems based on ROS and the open-source autonomous driving software Autoware, together with node development, mapping and localization, path planning, lateral control, autonomous driving sensors, and CAN communication. The instructor, schedule, assessment, course policies, and progress are in the [course information document](Course-Information.md).

<br><a name="prerequisites"></a>
## ✅ Prerequisites

- Programming skills in Python, C, and C++
- A PC or virtual machine (e.g., VirtualBox) running Ubuntu 18.04

- **💻 Development Environment**

| Tool | Company |  OS  | Notes |
|:-----|:-------:|:----:|:------|
|ROS Melodic|Open Robotics|Ubuntu 18.04|robot software framework|
|Autoware.ai 1.14.0|The Autoware Foundation|Ubuntu 18.04|open-source autonomous driving software|
|Qt 5.12.12|The Qt Company|Ubuntu 18.04|build dependency of Autoware, Qt Creator|
|Visual Studio Code|Microsoft|Ubuntu 18.04|ROS node development|

<br><a name="repository-structure"></a>
## 🗂 Repository Structure

```plaintext
25Spring_Autonomous-Mobile-Robot
├── Lecture-Notes               Lecture notes and example code by week
├── images                      Figures for the lecture notes
├── Course-Information.ko.md    Course operations
├── Course-Information.md
├── LICENSE                     Documentation license (CC BY-NC-SA 4.0)
├── LICENSE-CODE                Code license (PolyForm Noncommercial 1.0.0)
├── README.ko.md
└── README.md
```

The units are listed in the [`Lecture-Notes` index](Lecture-Notes/README.md).

<br><a name="license"></a>
## 🤝 License

The documents in this repository (lecture notes, explanations, and summary sheets) are licensed under [CC BY-NC-SA 4.0](LICENSE), and the code under [PolyForm Noncommercial 1.0.0](LICENSE-CODE). Figures and example code taken from lecture slides and textbooks, as well as assignment handouts and template code distributed by the course, belong to their original authors and are not covered by these licenses.

---
