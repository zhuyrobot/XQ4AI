#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QtWidgets/QtWidgets>
#include <QtSerialPort/QtSerialPort>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
using namespace std;
//using namespace cv;

class XQ4RC : public QWidget
{
public:
	struct XQFrame
	{
		int status;//小车状态: 0未初始化, 1正常, -1表示异常
		float power;//电源电压: 9~13V
		float theta;//方位角:0~360Deg
		int encoder_ppr;//车轮一转对应的编码器个数
		int encoder_delta_r;//右轮编码器增量: 单位个
		int encoder_delta_l;//左轮编码器增量: 单位个
		int encoder_delta_car;//两车轮中心位移: 单位个
		int omga_r;//右轮转速: 个每秒
		int omga_l;//左轮转速: 个每秒
		float distance1;//第一个超声模块距离值: 单位cm
		float distance2;//第二个超声模块距离值: 单位cm
		float distance3;//第三个超声模块距离值: 单位cm
		float distance4;//第四个超声模块距离值: 单位cm
		float imudata[9];//mpu9250 9轴数据
		uint timestamp;//时间戳
		string print(string savePath = "")
		{
			string str;
			str += fmt::format("status: {}\n", status);
			str += fmt::format("power: {}\n", power);
			str += fmt::format("theta: {}\n", theta);
			str += fmt::format("encoder_ppr: {}\n", encoder_ppr);
			str += fmt::format("encoder_delta_r: {}\n", encoder_delta_r);
			str += fmt::format("encoder_delta_l: {}\n", encoder_delta_l);
			str += fmt::format("encoder_delta_car: {}\n", encoder_delta_car);
			str += fmt::format("omga_r: {}\n", omga_r);
			str += fmt::format("omga_l: {}\n", omga_l);
			str += fmt::format("distance1: {}\n", distance1);
			str += fmt::format("distance2: {}\n", distance2);
			str += fmt::format("distance3: {}\n", distance3);
			str += fmt::format("distance4: {}\n", distance4);
			for (int k = 0; k < 9; ++k) str += fmt::format("imudata[{}]: {}\n", k, imudata[k]);
			str += fmt::format("timestamp: {}\n", timestamp);
			return str;
		}
	};

private:
	QSerialPort sport;
	const char heads[3] = { char(0xcd), char(0xeb), char(0xd7) };
	void setMode(char mode/*T/R/I*/)//T=debug   R=work   I=reset
	{
		if (!sport.isOpen()) { spdlog::critical("Serial port not openned: mode={}", mode); return; }
		char data[5] = { heads[0], heads[1], heads[2], char(0x01), mode };
		sport.write(data, 5);
	}
	void runCar(char action/*f/b/s/c/d*/, char velocity/*0~100*/)//f=move ahead   b=move back   s=brake   c=turn left   d=turn right
	{
		if (!sport.isOpen()) { spdlog::critical("Serial port not openned: action={}   velocity={}", action, int(velocity)); return; }
		char data[6] = { heads[0], heads[1], heads[2], char(0x02), action, velocity };
		sport.write(data, 6);
	}
	void runMotor(char action1/*F/B/S*/, char action2/*F/B/S*/, char velocity1/*0~100*/, char velocity2/*0~100*/)
	{
		if (!sport.isOpen()) { spdlog::critical("Serial port not openned: action1={}   action2={}   velocity1={}   velocity2={}", action1, action2, int(velocity1), int(velocity2)); return; }
		char data[13] = { heads[0], heads[1], heads[2], char(0x09), char(0x74), action1, action2, 0x53, 0x53, velocity1, velocity2, char(0x00), char(0x00) };
		sport.write(data, 13);
	}
	void runSensor(int action/*0=CalibIMU 1=EnableIMU 2=DisableIMU*/)
	{
		if (!sport.isOpen()) { spdlog::critical("Serial port not openned: action={}", action == 0 ? "CalibIMU" : action == 1 ? "EnableIR" : "DisableIR"); return; }
		char data[6] = { heads[0], heads[1], heads[2], action == 0 ? char(0x01) : char(0x02), action == 0 ? char(0x43) : char(0x44), action == 2 ? char(0x00) : char(0x01) };
		sport.write(data, action == 0 ? 5 : 6);
	}

private:
	QByteArray serArr;
	int readPos = 0;
	const int headSize = 4;
	const int itemSize = sizeof(int);
	const int itemSizeEx = itemSize + 1;
	const int itemCount = sizeof(XQFrame) / itemSize;
	const int dataSize = itemCount * itemSizeEx;
	const int frameSize = dataSize + headSize;
	const int maxArrSize = frameSize * 50 * 60; //Allow to lose several frames every one minute
	void sport_readyRead()
	{
		//1.Reset system
		if (serArr.size() > maxArrSize) { serArr.clear(); readPos = 0; }

		//2.Read serialport
		serArr.push_back(sport.readAll());

		//3.No need to read if less than one full frame
		if (serArr.size() < readPos + frameSize) { spdlog::warn("frameSize is small and this should not occur often"); return; }
		if (serArr.size() < 2 * frameSize) { spdlog::warn("Intial frameSize must be double for maxArrSize switch"); return; }

		//4.Find data head
		int curPos = readPos;
		for (; curPos < serArr.size(); ++curPos)
		{
#if 1
			if ((serArr[curPos] == heads[0] && serArr[curPos + 1] != heads[1] && serArr[curPos + 2] != heads[2]) ||
				(serArr[curPos] != heads[0] && serArr[curPos + 1] == heads[1] && serArr[curPos + 2] != heads[2]) ||
				(serArr[curPos] != heads[0] && serArr[curPos + 1] != heads[1] && serArr[curPos + 2] == heads[2]))
				spdlog::warn("data transition may have some problems and timestamp={}s", time(0));
#endif
			if (serArr[curPos] != heads[0]) continue;
			if (serArr[curPos + 1] != heads[1]) continue;
			if (serArr[curPos + 2] != heads[2]) continue;
			curPos += headSize;//Let curPos be dataPos if find the header
			break;
		}

		//5.No need to read if less than one data frame
		if (serArr.size() < curPos + dataSize) { spdlog::warn("dataSize is small and this should not occur often"); return; }

		//6.Read data
		XQFrame frame;
		for (int k = 0; k < itemCount; ++k) memcpy((int*)&frame + k, serArr.data() + curPos + itemSizeEx * k, itemSize);
		readPos = curPos + dataSize; //cout << endl << frame.print() << endl << endl;
		plainTextEditCarStatus->setPlainText(frame.print().c_str());
	}

public:
	static void RunMe(int argc = 0, char** argv = 0) { QApplication app(argc, argv); XQ4RC me; me.show(); app.exec(); }
	XQ4RC(QWidget* parent = 0) : QWidget(parent)
	{
		//0.Basic settting
		this->setWindowTitle("Super Cube");
		this->setMinimumSize(QSize(1280, 720));
		this->setFont(QFont("", 15, QFont::Thin));
		connect(&sport, &QSerialPort::readyRead, this, &XQ4RC::sport_readyRead);

		//1.Group1 setting
		gridLayoutMain->addWidget(comboBoxPorts, 0, 0, 1, 2);
		gridLayoutMain->addWidget(pushButtonOpen, 0, 2, 1, 1);
		gridLayoutMain->addWidget(spinBoxLinearSpeed, 1, 0);
		gridLayoutMain->addWidget(spinBoxAngularSpeed, 1, 1);
		gridLayoutMain->addWidget(spinBoxBrakeSpeed, 1, 2);
		gridLayoutMain->addWidget(pushButtonMoveHead, 2, 1);
		gridLayoutMain->addWidget(pushButtonTurnLeft, 3, 0);
		gridLayoutMain->addWidget(comboBoxWorkMode, 3, 1);
		gridLayoutMain->addWidget(pushButtonTurnRight, 3, 2);
		gridLayoutMain->addWidget(pushButtonMoveBack, 4, 1);
		gridLayoutMain->addWidget(pushButtonCalibIMU, 5, 1, 4, 1);
		gridLayoutMain->addWidget(pushButtonEnableIR, 5, 0);
		gridLayoutMain->addWidget(pushButtonDisableIR, 5, 2);
		gridLayoutMain->addWidget(pushButtonPlusMotor1, 6, 0);
		gridLayoutMain->addWidget(pushButtonMinusMotor1, 6, 2);
		gridLayoutMain->addWidget(pushButtonPlusMotor2, 7, 0);
		gridLayoutMain->addWidget(pushButtonMinusMotor2, 7, 2);
		gridLayoutMain->addWidget(pushButtonPlusMotors, 8, 0);
		gridLayoutMain->addWidget(pushButtonMinusMotors, 8, 2);
		gridLayoutMain->addWidget(pushButtonEnableROS2, 9, 0, 1, 3);
		gridLayoutMain->addWidget(plainTextEditCarStatus, 0, 3, 10, 1);
		{
			//1.GetAllSerPorts
			QList<QSerialPortInfo> listSerialPortInfo = QSerialPortInfo::availablePorts();
			for (int k = 0; k < listSerialPortInfo.size(); ++k) comboBoxPorts->addItem(listSerialPortInfo[k].portName());

			//2.FillOptions
			spinBoxLinearSpeed->setRange(0, 100); spinBoxLinearSpeed->setValue(10);
			spinBoxAngularSpeed->setRange(0, 100); spinBoxAngularSpeed->setValue(10);
			spinBoxBrakeSpeed->setRange(0, 100); spinBoxBrakeSpeed->setValue(10);
			comboBoxWorkMode->addItems(QStringList() << "DBG" << "Run" << "Reset");

			//3.AdjustGUI
			QList<QWidget*> children = this->findChildren<QWidget*>();
			for (int k = 0; k < children.size(); ++k) children[k]->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
			for (int k = 2; k < 5; ++k) gridLayoutMain->setRowStretch(k, 1);
			gridLayoutMain->setColumnStretch(3, 1);
			connect(pushButtonMoveHead, &QPushButton::pressed, [this]()->void { runCar('f', char(spinBoxLinearSpeed->value())); });
			connect(pushButtonMoveBack, &QPushButton::pressed, [this]()->void { runCar('b', char(spinBoxLinearSpeed->value())); });
			connect(pushButtonTurnLeft, &QPushButton::pressed, [this]()->void { runCar('c', char(spinBoxAngularSpeed->value())); });
			connect(pushButtonTurnRight, &QPushButton::pressed, [this]()->void { runCar('d', char(spinBoxAngularSpeed->value())); });
			connect(pushButtonMoveHead, &QPushButton::released, [this]()->void { runCar('s', char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonMoveBack, &QPushButton::released, [this]()->void { runCar('s', char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonTurnLeft, &QPushButton::released, [this]()->void { runCar('s', char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonTurnRight, &QPushButton::released, [this]()->void { runCar('s', char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonCalibIMU, &QPushButton::pressed, [this]()->void { runSensor(0); });
			connect(pushButtonEnableIR, &QPushButton::pressed, [this]()->void { runSensor(1); });
			connect(pushButtonDisableIR, &QPushButton::pressed, [this]()->void { runSensor(2); });
			connect(pushButtonPlusMotor1, &QPushButton::pressed, [this]()->void { runMotor('F', 'S', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonMinusMotor1, &QPushButton::pressed, [this]()->void { runMotor('B', 'S', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonPlusMotor2, &QPushButton::pressed, [this]()->void { runMotor('S', 'F', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonMinusMotor2, &QPushButton::pressed, [this]()->void { runMotor('S', 'B', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonPlusMotors, &QPushButton::pressed, [this]()->void { runMotor('F', 'F', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonMinusMotors, &QPushButton::pressed, [this]()->void { runMotor('B', 'B', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonPlusMotor1, &QPushButton::released, [this]()->void { runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonMinusMotor1, &QPushButton::released, [this]()->void { runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonPlusMotor2, &QPushButton::released, [this]()->void { runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonMinusMotor2, &QPushButton::released, [this]()->void { runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonPlusMotors, &QPushButton::released, [this]()->void { runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonMinusMotors, &QPushButton::released, [this]()->void { runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(comboBoxWorkMode, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)->void { char modes[3] = { 'T', 'R', 'I' }; setMode(modes[index]); });
			connect(pushButtonOpen, &QPushButton::clicked, [this]()->void
				{
					if (sport.isOpen())
					{
						sport.close();
						pushButtonOpen->setText("Open");
						spdlog::info("{} closed with success", sport.portName().toStdString());
					}
					else
					{
						sport.setPort(QSerialPortInfo(comboBoxPorts->currentText()));
						sport.setBaudRate(QSerialPort::Baud115200);
						sport.setDataBits(QSerialPort::Data8);
						sport.setStopBits(QSerialPort::OneStop);
						sport.setParity(QSerialPort::NoParity);
						if (sport.open(QIODevice::ReadWrite))
						{
							pushButtonOpen->setText("Close");
							spdlog::info("{} openned with success", sport.portName().toStdString());
						}
						else
						{
							spdlog::error("{} openned with failure", sport.portName().toStdString());
							QMessageBox::information(this, "", sport.portName() + " openned with failure");
						}
					}
				});
		}
	}

public:
	QGridLayout* gridLayoutMain = new QGridLayout(this);
	QComboBox* comboBoxPorts = new QComboBox(this);
	QPushButton* pushButtonOpen = new QPushButton("Open", this);
	QSpinBox* spinBoxLinearSpeed = new QSpinBox(this);
	QSpinBox* spinBoxAngularSpeed = new QSpinBox(this);
	QSpinBox* spinBoxBrakeSpeed = new QSpinBox(this);
	QComboBox* comboBoxWorkMode = new QComboBox(this);
	QPushButton* pushButtonMoveHead = new QPushButton("Move head", this);
	QPushButton* pushButtonMoveBack = new QPushButton("Move back", this);
	QPushButton* pushButtonTurnLeft = new QPushButton("Turn left", this);
	QPushButton* pushButtonTurnRight = new QPushButton("Turn right", this);
	QPushButton* pushButtonCalibIMU = new QPushButton("Calib IMU", this);
	QPushButton* pushButtonEnableIR = new QPushButton("Enable IR", this);
	QPushButton* pushButtonDisableIR = new QPushButton("Disable IR", this);
	QPushButton* pushButtonPlusMotor1 = new QPushButton("Plus motorR", this);
	QPushButton* pushButtonMinusMotor1 = new QPushButton("Minus motorR", this);
	QPushButton* pushButtonPlusMotor2 = new QPushButton("Plus motorL", this);
	QPushButton* pushButtonMinusMotor2 = new QPushButton("Minus motorL", this);
	QPushButton* pushButtonPlusMotors = new QPushButton("Plus motors", this);
	QPushButton* pushButtonMinusMotors = new QPushButton("Minus motors", this);
	QPushButton* pushButtonEnableROS2 = new QPushButton("Start ROS2", this);
	QPlainTextEdit* plainTextEditCarStatus = new QPlainTextEdit("", this);
};

int main(int argc, char** argv) { XQ4RC::RunMe(argc, argv); return 0; }