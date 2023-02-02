<img src="README.assets/openxglogo.png" alt="OpenXG" style="zoom: 100;" />

# 1. Generals
OpenXG is a series of 5G/6G-oriented open source projects developed by Open Source Radio Access Network Community. Currently, the community has developed the project from five aspects, including:
* Core network projects, 3GPP R15/R16 specification compliant network functions have been developed, including AMF, SMF, NRF, UPF, AUSF, UDM, AUSF, NWDAF, etc. Message level stateless mechanism is introduced in to the core network to make it better adapted to the cloud environment.
* Open hardware reference design, the reference design of heterogeneous computing hardware, white-box radio frontend, and open source UE hardware are also opened to the community.
* Radio access network projects, currently CU, DU and UE protocol software are developed, which is mainly compliant with 3GPP R15/R16 specifications. The software is developed by a modular-design, and which is flexible to customize to different vertical scenarios.
* AI-enabled management projects, three kinds of projects are developed under this category, including the AIEngine, which is aimed to provide the core AI capability for the management, the network measurement, and computing network for cloud-edge-end collaborative. 
* Uses cases and applications in verticals, some key use cases when applying OpenXG to vertical scenarios are also developed as a reference for the community.

# 2. License
The OpenXG series projects are distributed under OS-RAN license, which is derived from the Fair, Reasonable, and Non-Discriminatory principle. See [online version for details](http://www.openxg.org.cn/?falu_69.html).

# 3. How to start

## Install dependencies
OpenXG RAN relies on some third-party projects or libraries, in order to boost your install, the dependencies are hosted in a seperate repo. So please following step to install the dependencies:
```
git clone http://git.opensource5g.org/openxg/OpenXG-Install.git
sudo ./install
```

## Build RAN executalbes
OXGRF radio frontend is supported by OpenXG RAN project, build ran using following command:
```
sudo ./build_ran --gNB -w OXGRF
```

