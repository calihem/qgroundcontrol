/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009, 2010 QGROUNDCONTROL/PIXHAWK PROJECT
<http://www.qgroundcontrol.org>
<http://pixhawk.ethz.ch>

This file is part of the QGROUNDCONTROL project

    QGROUNDCONTROL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QGROUNDCONTROL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Implementation of class MainWindow
 *   @author Lorenz Meier <mail@qgroundcontrol.org>
 */

#include <QSettings>
#include <QDockWidget>
#include <QNetworkInterface>
#include <QMessageBox>
#include <QDebug>
#include <QTimer>
#include <QHostInfo>

#include "MG.h"
#include "ProtocolStack.h"
#include "MAVLinkSimulationLink.h"
#include "SerialLink.h"
#include "UDPLink.h"
#include "MAVLinkProtocol.h"
#include "CommConfigurationWindow.h"
#include "WaypointList.h"
#include "MainWindow.h"
#include "JoystickWidget.h"
#include "GAudioOutput.h"

// FIXME Move
#include "PxQuadMAV.h"
#include "SlugsMAV.h"


#include "LogCompressor.h"

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	settings()
{
	hide();

	// Setup user interface
	ui.setupUi(this);

	// Add link actions
	QList<int> linkIDs( ProtocolStack::instance().getLinkIDs() );
	foreach (int linkID, linkIDs)
	{
		addLinkAction(linkID);
	}

	buildWidgets();

	connectWidgets();

	arrangeCenterStack();

	configureWindowName();

	// Add status bar
	setStatusBar(createStatusBar());

	// Set the application style (not the same as a style sheet)
	// Set the style to Plastique
	qApp->setStyle("plastique");

	// Set style sheet as last step
	reloadStylesheet();

	// Connect actions
	connectActions();

	// Load widgets and show application window
	loadWidgets();

	// Adjust the size
	adjustSize();
}

MainWindow::~MainWindow()
{
    delete statusBar;
    statusBar = NULL;
}


void MainWindow::buildWidgets()
{
	QStringList* acceptList = new QStringList();
	acceptList->append("roll IMU");
	acceptList->append("pitch IMU");
	acceptList->append("yaw IMU");
	acceptList->append("rollspeed IMU");
	acceptList->append("pitchspeed IMU");
	acceptList->append("yawspeed IMU");

	QStringList* acceptList2 = new QStringList();
	acceptList2->append("Battery");
	acceptList2->append("Pressure");


	linechartWidget       = new Linecharts(this);
	controlWidget         = new UASControlWidget(this);
	listWidget            = new UASListWidget(this);
	waypointWidget        = new WaypointList(this, NULL);
	infoWidget            = new UASInfoWidget(this);
	detectionWidget       = new ObjectDetectionView("patterns", this);
	hudWidget             = new HUD(640, 480, this);
	debugConsoleWidget    = new DebugConsole(this);
	mapWidget             = new MapWidget(this);
	protocolWidget        = new XMLCommProtocolWidget(this);
	parameterWidget       = new ParameterInterface(this);
	watchdogControlWidget = new WatchdogControl(this);
	hsiWidget             = new HSIDisplay(this);
	hddWidget1            = new HDDisplay(acceptList, this);
	hddWidget2            = new HDDisplay(acceptList2, this);
	
	//TODO: Move
	joystick    = new JoystickInput();
}

void MainWindow::connectWidgets()
{
	connect( &ProtocolStack::instance(), SIGNAL(linkAdded(int)),
		this, SLOT(addLinkAction(int)) );

	if (linechartWidget)
	{
		connect(UASManager::instance(), SIGNAL(UASCreated(UASInterface*)),
			linechartWidget, SLOT(addSystem(UASInterface*)));
		connect(UASManager::instance(), SIGNAL(activeUASSet(int)),
			linechartWidget, SLOT(selectSystem(int)));
	}
	if (infoWidget)
	{
		ProtocolInterface *mavProtocol =
			ProtocolStack::instance().getProtocol(ProtocolStack::MAVLinkProtocol);
		connect(mavProtocol, SIGNAL(receiveLossChanged(int, float)),
			infoWidget, SLOT(updateSendLoss(int, float)));
	}
}

void MainWindow::arrangeCenterStack()
{
	centerStack = new QStackedWidget(this);
	if (linechartWidget) centerStack->addWidget(linechartWidget);
	if (protocolWidget) centerStack->addWidget(protocolWidget);
	if (mapWidget) centerStack->addWidget(mapWidget);
	if (hudWidget) centerStack->addWidget(hudWidget);

	setCentralWidget(centerStack);
}

void MainWindow::configureWindowName()
{
  QList<QHostAddress> hostAddresses = QNetworkInterface::allAddresses();
  QString windowname = qApp->applicationName() + " " + qApp->applicationVersion();
  bool prevAddr = false;

  windowname.append(" (" + QHostInfo::localHostName() + ": ");

  for (int i = 0; i < hostAddresses.size(); i++)
  {
      // Exclude loopback IPv4 and all IPv6 addresses
      if (hostAddresses.at(i) != QHostAddress("127.0.0.1") && !hostAddresses.at(i).toString().contains(":"))
      {
          if(prevAddr) windowname.append("/");
          windowname.append(hostAddresses.at(i).toString());
          prevAddr = true;
      }
  }

  windowname.append(")");

  setWindowTitle(windowname);

#ifndef Q_WS_MAC
  //qApp->setWindowIcon(QIcon(":/core/images/qtcreator_logo_128.png"));
#endif
}

QStatusBar* MainWindow::createStatusBar()
{
    QStatusBar* bar = new QStatusBar();
    /* Add status fields and messages */
    /* Enable resize grip in the bottom right corner */
    bar->setSizeGripEnabled(true);
    return bar;
}

void MainWindow::startVideoCapture()
{
    QString format = "bmp";
    QString initialPath = QDir::currentPath() + tr("/untitled.") + format;

    QString screenFileName = QFileDialog::getSaveFileName(this, tr("Save As"),
                                                          initialPath,
                                                          tr("%1 Files (*.%2);;All Files (*)")
                                                          .arg(format.toUpper())
                                                          .arg(format));
    delete videoTimer;
    videoTimer = new QTimer(this);
    //videoTimer->setInterval(40);
    //connect(videoTimer, SIGNAL(timeout()), this, SLOT(saveScreen()));
    //videoTimer->stop();
}

void MainWindow::stopVideoCapture()
{
    videoTimer->stop();

    // TODO Convert raw images to PNG
}

void MainWindow::saveScreen()
{
    QPixmap window = QPixmap::grabWindow(this->winId());
    QString format = "bmp";

    if (!screenFileName.isEmpty())
    {
        window.save(screenFileName, format.toAscii());
    }
}

void MainWindow::reloadStylesheet()
{
    // Load style sheet
    QFile* styleSheet = new QFile(QCoreApplication::applicationDirPath() + "/qgroundcontrol.css");
    if (!styleSheet->exists())
    {
        styleSheet = new QFile(":/images/style-mission.css");
    }
    if (styleSheet->open(QIODevice::ReadOnly | QIODevice::Text)) {
        QString style = QString(styleSheet->readAll());
        style.replace("ICONDIR", QCoreApplication::applicationDirPath()+ "/images/");
        qApp->setStyleSheet(style);
    } else {
        qDebug() << "Style not set:" << styleSheet->fileName() << "opened: " << styleSheet->isOpen();
    }
    delete styleSheet;
}

void MainWindow::showStatusMessage(const QString& status, int timeout)
{
    statusBar->showMessage(status, timeout);
}

void MainWindow::showStatusMessage(const QString& status)
{
    statusBar->showMessage(status, 5);
}

/**
* @brief Create all actions associated to the main window
*
**/
void MainWindow::connectActions()
{
    // Connect actions from ui
    connect(ui.actionAdd_Link, SIGNAL(triggered()), this, SLOT(invokeCommConfigDialog()));

    // Connect internal actions
    connect(UASManager::instance(), SIGNAL(UASCreated(UASInterface*)), this, SLOT(UASCreated(UASInterface*)));

    // Connect user interface controls
    connect(ui.actionLiftoff, SIGNAL(triggered()), UASManager::instance(), SLOT(launchActiveUAS()));
    connect(ui.actionLand, SIGNAL(triggered()), UASManager::instance(), SLOT(returnActiveUAS()));
    connect(ui.actionEmergency_Land, SIGNAL(triggered()), UASManager::instance(), SLOT(stopActiveUAS()));
    connect(ui.actionEmergency_Kill, SIGNAL(triggered()), UASManager::instance(), SLOT(killActiveUAS()));

    connect(ui.actionConfiguration, SIGNAL(triggered()), UASManager::instance(), SLOT(configureActiveUAS()));

    // User interface actions
    connect(ui.actionPilotView, SIGNAL(triggered()), this, SLOT(loadPilotView()));
    connect(ui.actionEngineerView, SIGNAL(triggered()), this, SLOT(loadEngineerView()));
    connect(ui.actionOperatorView, SIGNAL(triggered()), this, SLOT(loadOperatorView()));
    connect(ui.actionSettingsView, SIGNAL(triggered()), this, SLOT(loadSettingsView()));
    connect(ui.actionShow_full_view, SIGNAL(triggered()), this, SLOT(loadAllView()));
    connect(ui.actionShow_MAVLink_view, SIGNAL(triggered()), this, SLOT(loadMAVLinkView()));
    connect(ui.actionStyleConfig, SIGNAL(triggered()), this, SLOT(reloadStylesheet()));

    // Joystick configuration
    connect(ui.actionJoystickSettings, SIGNAL(triggered()), this, SLOT(configure()));
}

void MainWindow::configure()
{
    joystickWidget = new JoystickWidget(joystick, this);
}

void MainWindow::invokeCommConfigDialog()
{
	CommConfigurationWindow* commWidget(0);

	QAction *senderAction = dynamic_cast<QAction*>( sender() );
	if (senderAction)
	{
		LinkInterface *link = dynamic_cast<LinkInterface*>( senderAction->parent() );
		if (link) //create configuration dialog for already existing link
		{
			ProtocolInterface *protocol = ProtocolStack::instance().getProtocolForLink( link->getID() );
			commWidget = new CommConfigurationWindow(link, protocol, this);
		}
	}
	//create configuration widget for new link
	if (!commWidget) commWidget = new CommConfigurationWindow(NULL, NULL, this);

	if (!commWidget)
	{
		qDebug("Creation of communication configuration dialog failed");
		return;
	}

	//configure configuration dialog
	commWidget->setAttribute(Qt::WA_DeleteOnClose);
	commWidget->show();
	commWidget->raise();
	commWidget->activateWindow();
}

void MainWindow::addLinkAction(int linkID)
{
	LinkInterface *link = ProtocolStack::instance().getLink(linkID);
	if (!link) return;
	
	QAction *linkAction = new QAction(QIcon(":/images/devices/network-wireless.svg"), "", link);
	linkAction->setText( tr("Configure ") + link->getName() );
	linkAction->setStatusTip( tr("Configure ") + link->getName() );
	connect(linkAction, SIGNAL(triggered()), this, SLOT(invokeCommConfigDialog()));
	ui.menuNetwork->addAction(linkAction);

	// Special case for simulationlink
	MAVLinkSimulationLink* sim = dynamic_cast<MAVLinkSimulationLink*>(link);
	if (sim)
	{
		connect(ui.actionSimulate, SIGNAL(triggered(bool)),
			sim, SLOT(connectLink(bool)));
	}
}

void MainWindow::UASCreated(UASInterface* uas)
{
    // Connect the UAS to the full user interface
    //ui.menuConnected_Systems->addAction(QIcon(":/images/mavs/generic.svg"), tr("View ") + uas->getUASName(), uas, SLOT(setSelected()));

    // FIXME Should be not inside the mainwindow
    connect(uas, SIGNAL(textMessageReceived(int,int,QString)), debugConsoleWidget, SLOT(receiveTextMessage(int,int,QString)));

    // Health / System status indicator
    infoWidget->addUAS(uas);

    // UAS List
    listWidget->addUAS(uas);

    // Camera view
    //camera->addUAS(uas);

    // Revalidate UI
    // TODO Stylesheet reloading should in theory not be necessary
    reloadStylesheet();

    // Check which type this UAS is of
    PxQuadMAV* mav = dynamic_cast<PxQuadMAV*>(uas);
    if (mav) loadPixhawkView();
    SlugsMAV* mav2 = dynamic_cast<SlugsMAV*>(uas);
    if (mav2) loadSlugsView();
}

void MainWindow::clearView()
{ 
	// Halt HUD
	if (hudWidget) hudWidget->stop();
	if (linechartWidget) linechartWidget->setActive(false);
	if (hddWidget1) hddWidget1->stop();
	if (hddWidget2) hddWidget2->stop();
	if (hsiWidget) hsiWidget->stop();

	// Remove all dock widgets
	QObjectList childList( this->children() );
	
	QObjectList::iterator i;
	QDockWidget* dockWidget;
	for (i = childList.begin(); i != childList.end(); ++i)
	{
		dockWidget = dynamic_cast<QDockWidget*>(*i);
		if (dockWidget)
		{
			// Hide child widget of dock widget
			dockWidget->widget()->setVisible(false);
			// Remove and delete dock widget
			this->removeDockWidget(dockWidget);
			delete dockWidget;
		}
	}
}

void MainWindow::loadSlugsView()
{
	//TODO set up own view
	loadPixhawkView();
}

void MainWindow::loadPixhawkView()
{
	clearView();
	// Engineer view, used in EMAV2009

	// LINE CHART
	if (linechartWidget)
	{
		linechartWidget->setActive(true);
		centerStack->setCurrentWidget(linechartWidget);
	}
	// UAS CONTROL
	if (controlWidget)
	{
		QDockWidget* container1 = new QDockWidget(tr("Control"), this);
		container1->setWidget(controlWidget);
		addDockWidget(Qt::LeftDockWidgetArea, container1);
	}
	// UAS LIST
	if (listWidget)
	{
		QDockWidget* container4 = new QDockWidget(tr("Unmanned Systems"), this);
		container4->setWidget(listWidget);
		addDockWidget(Qt::BottomDockWidgetArea, container4);
	}
	// UAS STATUS
	if (infoWidget)
	{
		QDockWidget* container3 = new QDockWidget(tr("Status Details"), this);
		container3->setWidget(infoWidget);
		addDockWidget(Qt::LeftDockWidgetArea, container3);
	}
	// HORIZONTAL SITUATION INDICATOR
	if (hsiWidget)
	{
		QDockWidget* container6 = new QDockWidget(tr("Horizontal Situation Indicator"), this);
		container6->setWidget(hsiWidget);
		hsiWidget->start();
		addDockWidget(Qt::LeftDockWidgetArea, container6);
	}
	// WAYPOINT LIST
	if (waypointWidget)
	{
		QDockWidget* container5 = new QDockWidget(tr("Waypoint List"), this);
		container5->setWidget(waypointWidget);
		addDockWidget(Qt::BottomDockWidgetArea, container5);
	}
	// DEBUG CONSOLE
	if (debugConsoleWidget)
	{
		QDockWidget* container7 = new QDockWidget(tr("Communication Console"), this);
		container7->setWidget(debugConsoleWidget);
		addDockWidget(Qt::BottomDockWidgetArea, container7);
	}
	// ONBOARD PARAMETERS
	if (parameterWidget)
	{
		QDockWidget* containerParams = new QDockWidget(tr("Onboard Parameters"), this);
		containerParams->setWidget(parameterWidget);
		addDockWidget(Qt::RightDockWidgetArea, containerParams);
	}

	show();
}

void MainWindow::loadPilotView()
{
	clearView();

	// HEAD UP DISPLAY
	if (hudWidget)
	{
		centerStack->setCurrentWidget(hudWidget);
		hudWidget->start();
	}
	if (hddWidget1)
	{
		//connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), pfd, SLOT(setActiveUAS(UASInterface*)));
		QDockWidget* container1 = new QDockWidget(tr("Primary Flight Display"), this);
		container1->setWidget(hddWidget1);
		addDockWidget(Qt::RightDockWidgetArea, container1);
		hddWidget1->start();

	}
	if (hddWidget2)
	{
		QDockWidget* container2 = new QDockWidget(tr("Payload Status"), this);
		container2->setWidget(hddWidget2);
		addDockWidget(Qt::RightDockWidgetArea, container2);
		hddWidget2->start();
	}

	show();
}

void MainWindow::loadOperatorView()
{
	clearView();

	// MAP
	if (mapWidget)
	{
		centerStack->setCurrentWidget(mapWidget);
	}
	// UAS CONTROL
	if (controlWidget)
	{
		QDockWidget* container1 = new QDockWidget(tr("Control"), this);
		container1->setWidget(controlWidget);
		addDockWidget(Qt::LeftDockWidgetArea, container1);
	}
	// UAS LIST
	if (listWidget)
	{
		QDockWidget* container4 = new QDockWidget(tr("Unmanned Systems"), this);
		container4->setWidget(listWidget);
		addDockWidget(Qt::BottomDockWidgetArea, container4);
	}
	// UAS STATUS
	if (infoWidget)
	{
		QDockWidget* container3 = new QDockWidget(tr("Status Details"), this);
		container3->setWidget(infoWidget);
		addDockWidget(Qt::LeftDockWidgetArea, container3);
	}
	// WAYPOINT LIST
	if (waypointWidget)
	{
		QDockWidget* container5 = new QDockWidget(tr("Waypoint List"), this);
		container5->setWidget(waypointWidget);
		addDockWidget(Qt::BottomDockWidgetArea, container5);
	}
	// HORIZONTAL SITUATION INDICATOR
	if (hsiWidget)
	{
		QDockWidget* container7 = new QDockWidget(tr("Horizontal Situation Indicator"), this);
		container7->setWidget(hsiWidget);
		hsiWidget->start();
		addDockWidget(Qt::BottomDockWidgetArea, container7);
	}
	// OBJECT DETECTION
	if (detectionWidget)
	{
		QDockWidget* container6 = new QDockWidget(tr("Object Recognition"), this);
		container6->setWidget(detectionWidget);
		addDockWidget(Qt::RightDockWidgetArea, container6);
	}
	// PROCESS CONTROL
	if (watchdogControlWidget)
	{
		QDockWidget* pControl = new QDockWidget(tr("Process Control"), this);
		pControl->setWidget(watchdogControlWidget);
		addDockWidget(Qt::RightDockWidgetArea, pControl);
	}
	
	show();
}

void MainWindow::loadSettingsView()
{
	clearView();

	// LINE CHART
	if (linechartWidget)
	{
		linechartWidget->setActive(true);
		centerStack->setCurrentWidget(linechartWidget);
	}
    /*
    // COMM XML
    QDockWidget* container1 = new QDockWidget(tr("MAVLink XML to C Code Generator"), this);
    container1->setWidget(protocolWidget);
    addDockWidget(Qt::LeftDockWidgetArea, container1);*/

	// ONBOARD PARAMETERS
	if (parameterWidget)
	{
		QDockWidget* container6 = new QDockWidget(tr("Onboard Parameters"), this);
		container6->setWidget(parameterWidget);
		addDockWidget(Qt::RightDockWidgetArea, container6);
	}
	
	show();
}

// Engineer view, used in EMAV2009
void MainWindow::loadEngineerView()
{
	clearView();
    
	// LINE CHART
	if (linechartWidget)
	{
		linechartWidget->setActive(true);
		centerStack->setCurrentWidget(linechartWidget);
	}

	// UAS CONTROL
	if (controlWidget)
	{
		QDockWidget* container1 = new QDockWidget(tr("Control"), this);
		container1->setWidget(controlWidget);
		addDockWidget(Qt::LeftDockWidgetArea, container1);
	}

	// UAS LIST
	if (listWidget)
	{
		QDockWidget* container4 = new QDockWidget(tr("Unmanned Systems"), this);
		container4->setWidget(listWidget);
		addDockWidget(Qt::BottomDockWidgetArea, container4);
	}

	// UAS STATUS
	if (infoWidget)
	{
		QDockWidget* container3 = new QDockWidget(tr("Status Details"), this);
		container3->setWidget(infoWidget);
		addDockWidget(Qt::LeftDockWidgetArea, container3);
	}

	// WAYPOINT LIST
	if (waypointWidget)
	{
		QDockWidget* container5 = new QDockWidget(tr("Waypoint List"), this);
		container5->setWidget(waypointWidget);
		addDockWidget(Qt::BottomDockWidgetArea, container5);
	}

	// DEBUG CONSOLE
	if (debugConsoleWidget)
	{
		QDockWidget* container7 = new QDockWidget(tr("Communication Console"), this);
		container7->setWidget(debugConsoleWidget);
		addDockWidget(Qt::BottomDockWidgetArea, container7);
	}

	// ONBOARD PARAMETERS
	if (parameterWidget)
	{
		QDockWidget* containerParams = new QDockWidget(tr("Onboard Parameters"), this);
		containerParams->setWidget(parameterWidget);
		addDockWidget(Qt::RightDockWidgetArea, containerParams);
	}

	show();
}

void MainWindow::loadMAVLinkView()
{
	clearView();
	if (protocolWidget)
		centerStack->setCurrentWidget(protocolWidget);
	show();
}

void MainWindow::loadAllView()
{
	clearView();

	if (hddWidget1)
	{
		QDockWidget* containerPFD = new QDockWidget(tr("Primary Flight Display"), this);
		containerPFD->setWidget(hddWidget1);
		addDockWidget(Qt::RightDockWidgetArea, containerPFD);
		hddWidget1->start();
	}
	if (hddWidget2)
	{
		QDockWidget* containerPayload = new QDockWidget(tr("Payload Status"), this);
		containerPayload->setWidget(hddWidget2);
		addDockWidget(Qt::RightDockWidgetArea, containerPayload);
		hddWidget2->start();
	}
	// UAS CONTROL
	if (controlWidget)
	{
		QDockWidget* containerControl = new QDockWidget(tr("Control"), this);
		containerControl->setWidget(controlWidget);
		addDockWidget(Qt::LeftDockWidgetArea, containerControl);
	}
	// UAS LIST
	if (listWidget)
	{
		QDockWidget* containerUASList = new QDockWidget(tr("Unmanned Systems"), this);
		containerUASList->setWidget(listWidget);
		addDockWidget(Qt::BottomDockWidgetArea, containerUASList);
	}
	// UAS STATUS
	if (infoWidget)
	{
		QDockWidget* containerStatus = new QDockWidget(tr("Status Details"), this);
		containerStatus->setWidget(infoWidget);
		addDockWidget(Qt::LeftDockWidgetArea, containerStatus);
	}
	// WAYPOINT LIST
	if (waypointWidget)
	{
		QDockWidget* containerWaypoints = new QDockWidget(tr("Waypoint List"), this);
		containerWaypoints->setWidget(waypointWidget);
		addDockWidget(Qt::BottomDockWidgetArea, containerWaypoints);
	}
	// DEBUG CONSOLE
	if (debugConsoleWidget)
	{
		QDockWidget* containerComm = new QDockWidget(tr("Communication Console"), this);
		containerComm->setWidget(debugConsoleWidget);
		addDockWidget(Qt::BottomDockWidgetArea, containerComm);
	}
	// OBJECT DETECTION
	if (detectionWidget)
	{
		QDockWidget* containerObjRec = new QDockWidget(tr("Object Recognition"), this);
		containerObjRec->setWidget(detectionWidget);
		addDockWidget(Qt::RightDockWidgetArea, containerObjRec);
	}
	// LINE CHART
	if (linechartWidget)
	{
		linechartWidget->setActive(true);
		centerStack->setCurrentWidget(linechartWidget);
	}
	// ONBOARD PARAMETERS
	if (parameterWidget)
	{
		QDockWidget* containerParams = new QDockWidget(tr("Onboard Parameters"), this);
		containerParams->setWidget(parameterWidget);
		addDockWidget(Qt::RightDockWidgetArea, containerParams);
	}
	
	show();
}

void MainWindow::loadWidgets()
{
    //loadOperatorView();
    loadEngineerView();
    //loadPilotView();
}
