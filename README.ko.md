# [2025학년도 봄학기] 이동로봇운행시스템

![Last Commit](https://img.shields.io/github/last-commit/Choroning/25Spring_Autonomous-Mobile-Robot)
![Languages](https://img.shields.io/github/languages/top/Choroning/25Spring_Autonomous-Mobile-Robot)

이 레포지토리는 이동로봇운행시스템 과목의 강의 노트를 정리한 아카이브입니다.

*작성자: 박철원 (고려대학교(세종), 컴퓨터소프트웨어학과) - 2025년 기준 2학년*
<br><br>

## 📑 목차

- [레포지토리 소개](#about-this-repository)
- [강의 정보](#course-information)
- [사전 요구사항](#prerequisites)
- [레포지토리 구조](#repository-structure)
- [라이선스](#license)

---


<br><a name="about-this-repository"></a>
## 📝 레포지토리 소개

- `Lecture-Notes/`에는 주차별(`Week01`, `Week02` 등)로 그 주의 강의자료를 정리한 강의 노트가 있고, 5\~7주차에는 강의에서 다룬 ROS 노드 예제 코드가 함께 있습니다.
- 모든 강의 노트는 영어(`.md`)와 한국어(`.ko.md`) 파일이 짝을 이룹니다.

> **🤖 생성형 AI 도구 사용**
> 이 레포지토리는 문서화에 [Claude Code](https://claude.ai/download)를 사용합니다.

<br><a name="course-information"></a>
## 📚 강의 정보

- **학기:** 2025학년도 봄학기 (3월 - 6월)
- **소속:** 고려대학교(세종)

|학수번호      |강의명    |이수구분|교수자|개설학과|
|:----------:|:-------|:----:|:------:|:----------------|
|`DAMO425-00`|이동로봇운행시스템|전공선택|송영은 교수|미래모빌리티학과|

### 과목 개요

ROS와 오픈 소스 자율주행 소프트웨어 Autoware를 바탕으로 이동 로봇 운행 시스템의 구조와 노드 개발, 지도 작성과 위치 인식, 경로 계획, 횡방향 제어, 자율주행 센서와 CAN 통신을 이론과 실습으로 배우는 전공선택 과목입니다. 교수자, 수업 일정, 평가, 수업 운영 규정, 진도는 [강의 정보 문서](Course-Information.ko.md)에 정리했습니다.

<br><a name="prerequisites"></a>
## ✅ 사전 요구사항

- Python, C, C++ 프로그래밍 능력
- Ubuntu 18.04를 설치한 PC 또는 가상 머신(예: VirtualBox)

- **💻 개발 환경**

| 도구 | 회사 |  운영체제  | 비고 |
|:-----|:-------:|:----:|:------|
|ROS Melodic|Open Robotics|Ubuntu 18.04|로봇 소프트웨어 프레임워크|
|Autoware.ai 1.14.0|The Autoware Foundation|Ubuntu 18.04|오픈 소스 자율주행 소프트웨어|
|Qt 5.12.12|The Qt Company|Ubuntu 18.04|Autoware 빌드 의존성, Qt Creator|
|Visual Studio Code|Microsoft|Ubuntu 18.04|ROS 노드 개발|

<br><a name="repository-structure"></a>
## 🗂 레포지토리 구조

```plaintext
25Spring_Autonomous-Mobile-Robot
├── Lecture-Notes               주차별 강의 노트와 예제 코드
├── images                      강의 노트의 그림
├── Course-Information.ko.md    수업 운영 정보
├── Course-Information.md
├── LICENSE                     문서 라이선스 (CC BY-NC-SA 4.0)
├── LICENSE-CODE                코드 라이선스 (PolyForm Noncommercial 1.0.0)
├── README.ko.md
└── README.md
```

단원 목록은 [`Lecture-Notes` 색인](Lecture-Notes/README.ko.md)에 있습니다.

<br><a name="license"></a>
## 🤝 라이선스

이 레포지토리의 문서(강의 노트, 해설, 요약지)는 [CC BY-NC-SA 4.0](LICENSE), 코드는 [PolyForm Noncommercial 1.0.0](LICENSE-CODE)에 따라 배포됩니다. 강의자료와 교재에서 가져온 그림과 예제 코드, 강의가 배포한 과제 원본과 템플릿 코드는 원저작자에게 권리가 있으며 이 라이선스의 대상이 아닙니다.

---
