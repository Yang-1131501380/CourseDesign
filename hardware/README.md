# Hardware

该目录用于存放项目公开同步的硬件设计资料，面向仓库展示与工程复现。

## 当前内容

- `CousrseDesign.eprj2`
  嘉立创EDA专业版工程文件，包含本项目硬件设计的核心工程数据。

## 系统连接框图

```text
                +----------------------+
                |        K230          |
                | target detect / dist |
                +----------+-----------+
                           |
                         USART6
                           |
                +----------v-----------+
                |      STM32F407       |
                | control / parser     |
                | attitude / follow    |
                +----+-----------+-----+
                     |           |
                  USART2       I2C1
                     |           |
          +----------v---+   +---v----------------+
          |   EMM_V5     |   | ICM42688 / QMC5883 |
          | yaw/pitch    |   | IMU / compass      |
          +--------------+   +--------------------+
                     |
                 track result
                     |
                +----v-----+
                |  chassis |
                | TB6612   |
                | encoder  |
                +----+-----+
                     |
                 DC motors

        LCD display / serial log -> status monitoring
```

## 说明

- 当前仓库仅同步项目主硬件设计文件，避免将本地参考资料、第三方开发板配套包和无关附件一并上传。
- 本地仍保留其他参考性硬件资料，但默认不再同步到 GitHub。

## 后续可补充内容

- 原理图导出 PDF
- PCB 截图或打样图
- BOM 清单
- 硬件模块说明文档
- 接线说明或系统连接框图细化版本
