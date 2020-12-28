#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QtWidgets/QtWidgets>
#include <QtSerialPort/QtSerialPort>
#include<opencv2/opencv.hpp>
#include<spdlog/spdlog.h>
using namespace std;
using namespace cv;

#define DISABLE 0
#define ENABLE 1

class XQ4RC : public QWidget
{
public:
	struct XQFrameEx
	{
		int status; char c1;//小车状态: 0未初始化, 1正常, -1表示异常
		float power; char c2;//电源电压: 9~13V
		float theta; char c3;//方位角:0~360Deg
		int encoder_ppr; char c4;//车轮一转对应的编码器个数
		int encoder_delta_r; char c5;//右轮编码器增量: 单位个
		int encoder_delta_l; char c6;//左轮编码器增量: 单位个
		int encoder_delta_car; char c7;//两车轮中心位移: 单位个
		int omga_r; char c8;//右轮转速: 个每秒
		int omga_l; char c9;//左轮转速: 个每秒
		float distance1; char c10;//第一个超声模块距离值: 单位cm
		float distance2; char c11;//第二个超声模块距离值: 单位cm
		float distance3; char c12;//第三个超声模块距离值: 单位cm
		float distance4; char c13;//第四个超声模块距离值: 单位cm
		float _imudata[45];//mpu9250 9轴数据
		uint timestamp; char c23;//时间戳
		float* ImuData[9] = { _imudata, _imudata + 5, _imudata + 10, _imudata + 15, _imudata + 20, _imudata + 25, _imudata + 30, _imudata + 35, _imudata + 40 };
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
			for (int k = 0; k < 9; ++k) str += fmt::format("IMUData[{}]: {}\n", k, *(ImuData[k]));
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
	int lastHeadPos = -1;
	const int dataSize = 23 * 5;
	const int frameSize = dataSize + 4;
	const int maxArrSize = frameSize * 50 * 60; //Allow to lose several frames every one minute
//	void sport_readyRead()
//	{
//#if 0
//		//serArr.push_back(sport.readAll());
//		//char* data;
//		//if (serArr.size() > 1000)
//		//{
//		//	data = serArr.data();
//		//	cout << endl << 123 << endl;
//		//	getchar();
//		//}
//		//int x = 5;
//
//#else
//		//1.Reset system
//		if (serArr.size() > maxArrSize) { serArr.clear(); readPos = 0; lastHeadPos = -1; }
//		//2.Read serialport
//		serArr.push_back(sport.readAll());
//		//3.No need to read if less than one frame
//		if (serArr.size() < readPos + frameSize) return;
//		//4.Find data head
//		int k = lastHeadPos;
//		if (k == -1)
//		{
//			k = readPos;
//			for (; k < serArr.size(); ++k)
//			{
//				if (serArr[k] != heads[0]) continue;
//				if (serArr[k + 1] != heads[1]) continue;
//				if (serArr[k + 2] != heads[2]) continue;
//				break;
//			}
//		}
//		//5.No need to read if less than one frame
//		if (serArr.size() - k < dataSize)
//		{
//			lastHeadPos = k;
//			cout << endl << k << endl;
//			return;
//		}
//		//6.
//		XQFrameEx frame;
//		memcpy(&frame, serArr.data() + k + 4, dataSize);
//		readPos = k + frameSize;
//		lastHeadPos = -1;
//		plainTextEditCarStatus->setPlainText(frame.print().c_str());
//		cout << endl << frame.print() << endl << endl;
//#endif
//	}

	void sport_readyRead()
	{
		int i = 0, j = 0;
		int* receive_byte;
		static unsigned char last_str[2] = { 0x00, 0x00 };
		static unsigned char new_packed_ctr = DISABLE; //ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
		static int new_packed_ok_len = 0;              //包的理论长度
		static int new_packed_len = 0;                 //包的实际长度
		static unsigned char cmd_string_buf[512];
		unsigned char current_str = 0x00;
		const int cmd_string_max_size = 512;
		receive_byte = (int*)&car_status;
		//int ii=0;
		//boost::mutex::scoped_lock lock(mMutex);

		// if(len<119)
		// {
		// std::cout<<"len0:"<<len<<std::endl;
		//   current_str=data[0];
		//   std::cout<<(unsigned int)current_str<<std::endl;
		// }
		for (i = 0; i < len; i++)
		{
			current_str = data[i];
			// unsigned int temp=(unsigned int)current_str;
			// std::cout<<temp<<std::endl;
			//判断是否有新包头
			if (last_str[0] == 205 && last_str[1] == 235 && current_str == 215) //包头 205 235 215
			{
				//std::cout<<"runup1 "<<std::endl;
				new_packed_ctr = ENABLE;
				new_packed_ok_len = 0;
				new_packed_len = new_packed_ok_len;
				last_str[0] = last_str[1]; //保存最后两个字符，用来确定包头
				last_str[1] = current_str;
				continue;
			}
			last_str[0] = last_str[1]; //保存最后两个字符，用来确定包头
			last_str[1] = current_str;
			if (new_packed_ctr == ENABLE)
			{

				//获取包长度
				new_packed_ok_len = current_str;
				if (new_packed_ok_len > cmd_string_max_size)
					new_packed_ok_len = cmd_string_max_size; //包内容最大长度有限制
				new_packed_ctr = DISABLE;
				//std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
			}
			else
			{
				//判断包当前大小
				if (new_packed_ok_len <= new_packed_len)
				{
					//std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
					//包长度已经大于等于理论长度，后续内容无效
					continue;
				}
				else
				{
					//获取包内容
					new_packed_len++;
					cmd_string_buf[new_packed_len - 1] = current_str;
					if (new_packed_ok_len == new_packed_len && new_packed_ok_len > 0)
					{
						// std::cout<<"runup4 "<<std::endl;
						//当前包已经处理完成，开始处理
						if (new_packed_ok_len == 115)
						{
							for (j = 0; j < 23; j++)
							{
								memcpy(&receive_byte[j], &cmd_string_buf[5 * j], 4);
							}
							mbUpdated = true;
						}
						else if (new_packed_ok_len == 95)
						{
							for (j = 0; j < 19; j++)
							{
								memcpy(&receive_byte[j], &cmd_string_buf[5 * j], 4);
							}
							mbUpdated = true;
						}
						if (mbUpdated)
						{
							for (j = 0; j < 7; j++)
							{
								if (cmd_string_buf[5 * j + 4] != 32)
								{
									//   std::cout<<"len:"<< len <<std::endl;
									//   std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
									//   for(j=0;j<115;j++)
									//   {
									//     current_str=cmd_string_buf[j];
									//     std::cout<<(unsigned int)current_str<<std::endl;
									//   }
									mbUpdated = false;
									car_status.encoder_ppr = 4 * 12 * 64;
									break;
								}
							}
						}
						if (mbUpdated)
						{
							base_time_ = ros::Time::now().toSec();
						}
						new_packed_ok_len = 0;
						new_packed_len = 0;
					}
				}
			}
		}


		return;
		//1.Reset system
		if (serArr.size() > maxArrSize) { serArr.clear(); readPos = 0; lastHeadPos = -1; }
		//2.Read serialport
		serArr.push_back(sport.readAll());
		//3.No need to read if less than one frame
		if (serArr.size() < readPos + frameSize) return;
		//4.Find data head
		int k = lastHeadPos;
		if (k == -1)
		{
			k = readPos;
			for (; k < serArr.size(); ++k)
			{
				if (serArr[k] != heads[0]) continue;
				if (serArr[k + 1] != heads[1]) continue;
				if (serArr[k + 2] != heads[2]) continue;
				break;
			}
		}
		//5.No need to read if less than one frame
		if (serArr.size() - k < dataSize)
		{
			lastHeadPos = k;
			cout << endl << k << endl;
			return;
		}
		//6.
		XQFrameEx frame;
		memcpy(&frame, serArr.data() + k + 4, dataSize);
		readPos = k + frameSize;
		lastHeadPos = -1;
		plainTextEditCarStatus->setPlainText(frame.print().c_str());
		cout << endl << frame.print() << endl << endl;
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