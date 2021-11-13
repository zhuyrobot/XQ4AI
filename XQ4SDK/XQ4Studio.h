#include "XQ4ROS.h"
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QtWidgets/QtWidgets>
#include <QtSerialPort/QtSerialPort>

class XQ4Studio : public QWidget
{
public:
	XQ4IO xq4io;

public:
	static void RunMe(int argc = 0, char** argv = 0) { QApplication app(argc, argv); XQ4Studio me; me.show(); app.exec(); }
	XQ4Studio(QWidget* parent = 0) : QWidget(parent)
	{
		//0.Basic settting
		this->setWindowTitle(__FUNCTION__);
		this->setMinimumSize(QSize(1280, 720));
		this->setFont(QFont("", 15, QFont::Thin));

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
		gridLayoutMain->addWidget(pushButtonStartXQ4ROS, 9, 0, 1, 3);
		gridLayoutMain->addWidget(pushButtonStartRPLidarROS, 10, 0, 1, 3);
		gridLayoutMain->addWidget(pushButtonStartXQ4Simulator, 11, 0, 1, 3);
		gridLayoutMain->addWidget(plainTextEditCarStatus, 0, 3, 11, 2);
		gridLayoutMain->addWidget(comboBoxROS2CMD, 11, 3, 1, 1);
		gridLayoutMain->addWidget(pushButtonROS2Call, 11, 4, 1, 1);
		{
			//1.GetAllSerPorts
			QList<QSerialPortInfo> listSerialPortInfo = QSerialPortInfo::availablePorts();
			for (int k = 0; k < listSerialPortInfo.size(); ++k) comboBoxPorts->addItem(listSerialPortInfo[k].portName());

			//2.FillOptions
			spinBoxLinearSpeed->setRange(0, 100); spinBoxLinearSpeed->setValue(10);
			spinBoxAngularSpeed->setRange(0, 100); spinBoxAngularSpeed->setValue(10);
			spinBoxBrakeSpeed->setRange(0, 100); spinBoxBrakeSpeed->setValue(10);
			comboBoxWorkMode->addItems(QStringList() << "DBG" << "Run" << "Reset");
			comboBoxROS2CMD->setEditable(true);
			comboBoxROS2CMD->addItem("ros2 service call /XQ/ClosePort std_srvs/srv/Trigger \"{}\"");
			comboBoxROS2CMD->addItem("ros2 service call /XQ/OpenPort ams_xq/srv/XQ4Serve \"{cmd: 'COM2'}\"");
			comboBoxROS2CMD->addItem("ros2 service call /XQ/SetMode ams_xq/srv/XQ4Serve \"{cmd: 'T'}\"");
			comboBoxROS2CMD->addItem("ros2 service call /XQ/RunSensor ams_xq/srv/XQ4Serve \"{cmd: '1'}\"");
			comboBoxROS2CMD->addItem("ros2 service call -r 1 /XQ/RunMotor ams_xq/srv/XQ4Serve \"{cmd: 'FF22'}\"");
			comboBoxROS2CMD->addItem("ros2 topic pub -r 1 /XQ/RunCar std_msgs/msg/String \"{data: 'f2'}\"");

			//3.AdjustGUI
			QList<QWidget*> children = this->findChildren<QWidget*>();
			for (int k = 0; k < children.size(); ++k) children[k]->setSizePolicy(QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred));
			for (int k = 2; k < 5; ++k) gridLayoutMain->setRowStretch(k, 1);
			gridLayoutMain->setColumnStretch(3, 1);
			connect(pushButtonMoveHead, &QPushButton::pressed, [this]()->void { xq4io.runCar('f', char(spinBoxLinearSpeed->value())); });
			connect(pushButtonMoveBack, &QPushButton::pressed, [this]()->void { xq4io.runCar('b', char(spinBoxLinearSpeed->value())); });
			connect(pushButtonTurnLeft, &QPushButton::pressed, [this]()->void { xq4io.runCar('c', char(spinBoxAngularSpeed->value())); });
			connect(pushButtonTurnRight, &QPushButton::pressed, [this]()->void { xq4io.runCar('d', char(spinBoxAngularSpeed->value())); });
			connect(pushButtonMoveHead, &QPushButton::released, [this]()->void { xq4io.runCar('s', char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonMoveBack, &QPushButton::released, [this]()->void { xq4io.runCar('s', char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonTurnLeft, &QPushButton::released, [this]()->void { xq4io.runCar('s', char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonTurnRight, &QPushButton::released, [this]()->void { xq4io.runCar('s', char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonCalibIMU, &QPushButton::pressed, [this]()->void { xq4io.runSensor(0); });
			connect(pushButtonEnableIR, &QPushButton::pressed, [this]()->void { xq4io.runSensor(1); });
			connect(pushButtonDisableIR, &QPushButton::pressed, [this]()->void { xq4io.runSensor(2); });
			connect(pushButtonPlusMotor1, &QPushButton::pressed, [this]()->void { xq4io.runMotor('F', 'S', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonMinusMotor1, &QPushButton::pressed, [this]()->void { xq4io.runMotor('B', 'S', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonPlusMotor2, &QPushButton::pressed, [this]()->void { xq4io.runMotor('S', 'F', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonMinusMotor2, &QPushButton::pressed, [this]()->void { xq4io.runMotor('S', 'B', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonPlusMotors, &QPushButton::pressed, [this]()->void { xq4io.runMotor('F', 'F', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonMinusMotors, &QPushButton::pressed, [this]()->void { xq4io.runMotor('B', 'B', char(spinBoxLinearSpeed->value()), char(spinBoxAngularSpeed->value())); });
			connect(pushButtonPlusMotor1, &QPushButton::released, [this]()->void { xq4io.runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonMinusMotor1, &QPushButton::released, [this]()->void { xq4io.runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonPlusMotor2, &QPushButton::released, [this]()->void { xq4io.runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonMinusMotor2, &QPushButton::released, [this]()->void { xq4io.runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonPlusMotors, &QPushButton::released, [this]()->void { xq4io.runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(pushButtonMinusMotors, &QPushButton::released, [this]()->void { xq4io.runMotor('S', 'S', char(spinBoxBrakeSpeed->value()), char(spinBoxBrakeSpeed->value())); });
			connect(comboBoxWorkMode, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index)->void { char modes[3] = { 'T', 'R', 'I' }; xq4io.setMode(modes[index]); });
			connect(pushButtonROS2Call, &QPushButton::clicked, [this]()->void { QProcess::execute(comboBoxROS2CMD->currentText()); });
			connect(pushButtonOpen, &QPushButton::clicked, [this]()->void
				{
					if (xq4io.opened())
					{
						timerCarStatus->stop();
						xq4io.close();
						pushButtonOpen->setText("Open");
						spdlog::info("{} closed with success", xq4io.name());
					}
					else
					{
						if (xq4io.open(comboBoxPorts->currentText().toStdString()))
						{
							pushButtonOpen->setText("Close");
							spdlog::info("{} openned with success", xq4io.name());
							timerCarStatus->start(50);
						}
						else
						{
							spdlog::error("{} openned with failure", xq4io.name());
							QMessageBox::information(this, "", xq4io.name().c_str() + QString(" openned with failure"));
						}
					}
				});
			connect(timerCarStatus, &QTimer::timeout, [this]()->void 
				{
					XQ4IO::XQ4Frame* msg = 0;
					xq4io.getStatus(&msg);
					if (msg != 0) plainTextEditCarStatus->setPlainText(msg->print().c_str());
					else plainTextEditCarStatus->setPlainText(fmt::format("No data received {}", tsms).c_str());
				});
			connect(pushButtonStartXQ4ROS, &QPushButton::pressed, [this]()->void { });
			connect(pushButtonStartXQ4Simulator, &QPushButton::pressed, [this]()->void 
				{ 	
					QList<QSerialPortInfo> listSerialPortInfo = QSerialPortInfo::availablePorts();
					QStringList items; for (int k = 0; k < listSerialPortInfo.size(); ++k)items.push_back(listSerialPortInfo[k].portName());
					QString sport = QInputDialog::getItem(this, "", "Choose or input serial port", items);
					std::thread(&XQ4IO::XQ4Sim, sport.toStdString()).detach();
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
	QPushButton* pushButtonStartXQ4ROS = new QPushButton("Start XQ4ROS", this);
	QPushButton* pushButtonStartRPLidarROS = new QPushButton("Start RPLidarROS", this);
	QPushButton* pushButtonStartXQ4Simulator = new QPushButton("Start XQ4Simulator", this);
	QPlainTextEdit* plainTextEditCarStatus = new QPlainTextEdit("", this);
	QComboBox* comboBoxROS2CMD = new QComboBox(this);
	QPushButton* pushButtonROS2Call = new QPushButton("Call", this);
	QTimer* timerCarStatus = new QTimer(this);
};

//int main(int argc, char** argv) { XQ4Studio::RunMe(argc, argv); return 0; }