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
	settings(),
	centerStack(NULL)
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
	//FIXME: memory of acceptList will never be freed again
	QStringList* acceptList = new QStringList();
	acceptList->append("roll IMU");
	acceptList->append("pitch IMU");
	acceptList->append("yaw IMU");
	acceptList->append("rollspeed IMU");
	acceptList->append("pitchspeed IMU");
	acceptList->append("yawspeed IMU");

	//FIXME: memory of acceptList2 will never be freed again
	QStringList* acceptList2 = new QStringList();
	acceptList2->append("Battery");
	acceptList2->append("Pressure");

	// center widgets
	linechartWidget = new Linecharts(this);
	protocolWidget  = new XMLCommProtocolWidget(this);
	mapWidget       = new MapWidget(this);
	hudWidget       = new HUD(640, 480, this);

	// dock widgets
	controlDockWidget = new QDockWidget(tr("Control"), this);
	controlDockWidget->setWidget( new UASControlWidget(this) );

	listDockWidget = new QDockWidget(tr("Unmanned Systems"), this);
	listDockWidget->setWidget( new UASListWidget(this) );

	waypointDockWidget = new QDockWidget(tr("Waypoint List"), this);
	waypointDockWidget->setWidget( new WaypointList(this, NULL) );

	infoDockWidget = new QDockWidget(tr("Status Details"), this);
	infoDockWidget->setWidget( new UASInfoWidget(this) );

	detectionDockWidget = new QDockWidget(tr("Object Recognition"), this);
	detectionDockWidget->setWidget( new ObjectDetectionView("patterns", this) );

	debugConsoleDockWidget = new QDockWidget(tr("Communication Console"), this);
	debugConsoleDockWidget->setWidget( new DebugConsole(this) );

	parameterDockWidget = new QDockWidget(tr("Onboard Parameters"), this);
	parameterDockWidget->setWidget( new ParameterInterface(this) );

	watchdogControlDockWidget = new QDockWidget(tr("Process Control"), this);
	watchdogControlDockWidget->setWidget( new WatchdogControl(this) );

	hsiDockWidget = new QDockWidget(tr("Horizontal Situation Indicator"), this);
	hsiDockWidget->setWidget( new HSIDisplay(this) );

	hddDockWidget1 = new QDockWidget(tr("Primary Flight Display"), this);
	hddDockWidget1->setWidget( new HDDisplay(acceptList, this) );

	hddDockWidget2 = new QDockWidget(tr("Payload Status"), this);
	hddDockWidget2->setWidget( new HDDisplay(acceptList2, this) );

	//FIXME: free memory in destructor
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
	if (infoDockWidget && infoDockWidget->widget())
	{
		ProtocolInterface *mavProtocol =
			ProtocolStack::instance().getProtocol(ProtocolStack::MAVLinkProtocol);
		connect(mavProtocol, SIGNAL(receiveLossChanged(int, float)),
			infoDockWidget->widget(), SLOT(updateSendLoss(int, float)));
	}
}

void MainWindow::arrangeCenterStack()
{
	centerStack = new QStackedWidget(this);
	if (!centerStack) return;

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
	QFile styleSheetFile(QCoreApplication::applicationDirPath() + "/qgroundcontrol.css");
	if ( !styleSheetFile.exists() )
	{
		styleSheetFile.setFileName(":/images/style-mission.css");
	}
	if (styleSheetFile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QString style( styleSheetFile.readAll() );
		style.replace("ICONDIR", QCoreApplication::applicationDirPath()+ "/images/");
		qApp->setStyleSheet(style);
	}
	else
	{
		qDebug() << "Style not set:" << styleSheetFile.fileName() << "opened: " << styleSheetFile.isOpen();
	}
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
	CommConfigurationWindow* commWidget(NULL);

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
	if (!uas) return;

	// Connect the UAS to the full user interface
	//ui.menuConnected_Systems->addAction(QIcon(":/images/mavs/generic.svg"), tr("View ") + uas->getName(), uas, SLOT(setSelected()));

	// FIXME Should be not inside the mainwindow
	if ( debugConsoleDockWidget && debugConsoleDockWidget->widget() )
	{
		connect(uas, SIGNAL(textMessageReceived(int,int,QString)),
			debugConsoleDockWidget->widget(), SLOT(receiveTextMessage(int,int,QString)));
	}

	// Health / System status indicator
	if ( infoDockWidget && infoDockWidget->widget() )
	{
		UASInfoWidget* infoWidget = dynamic_cast<UASInfoWidget*>( infoDockWidget->widget() );
		if (infoWidget) infoWidget->addUAS(uas);
	}

	// UAS List
	if ( listDockWidget && listDockWidget->widget() )
	{
		UASListWidget* listWidget = dynamic_cast<UASListWidget*>( listDockWidget->widget() );
		if (listWidget) listWidget->addUAS(uas);
	}

	// Camera view
	//camera->addUAS(uas);

	// Check which type this UAS is of
	PxQuadMAV* mav = dynamic_cast<PxQuadMAV*>(uas);
	if (mav) loadPixhawkView();
	SlugsMAV* mav2 = dynamic_cast<SlugsMAV*>(uas);
	if (mav2) loadSlugsView();
}

void MainWindow::clearView()
{ 
	// Disable linechart
	if (linechartWidget) linechartWidget->setActive(false);
	// Halt HUD
	if (hudWidget) hudWidget->stop();
	// Halt HDDs
	if (hddDockWidget1 && hddDockWidget1->widget())
	{
		HDDisplay* hddWidget = dynamic_cast<HDDisplay*>( hddDockWidget1->widget() );
		if (hddWidget) hddWidget->stop();
	}
	if (hddDockWidget2 && hddDockWidget2->widget())
	{
		HDDisplay* hddWidget = dynamic_cast<HDDisplay*>( hddDockWidget2->widget() );
		if (hddWidget) hddWidget->stop();
	}
	// Halt HSI
	if (hsiDockWidget && hsiDockWidget->widget() )
	{
		HSIDisplay* hsi = dynamic_cast<HSIDisplay*>( hsiDockWidget->widget() );
		if (hsi) hsi->stop();
	}
	// Hide center widget
	if ( centerStack && centerStack->currentWidget() )
		centerStack->currentWidget()->hide();

	// Remove all dock widgets from main window
	QObjectList childList( this->children() );
	
	QObjectList::iterator i;
	QDockWidget* dockWidget;
	for (i = childList.begin(); i != childList.end(); ++i)
	{
		dockWidget = dynamic_cast<QDockWidget*>(*i);
		if (dockWidget)
		{
			// Remove dock widget from main window
			this->removeDockWidget(dockWidget);
		}
	}
}

void MainWindow::loadSlugsView()
{
	//TODO set up own view
	loadPixhawkView();
}

// Engineer view, used in EMAV2009
void MainWindow::loadPixhawkView()
{
	clearView();

	// LINE CHART
	if (linechartWidget && centerStack)
	{
		linechartWidget->setActive(true);
		centerStack->setCurrentWidget(linechartWidget);
		linechartWidget->show();
	}

	// UAS CONTROL
	if (controlDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, controlDockWidget);
		controlDockWidget->show();
	}
	// UAS LIST
	if (listDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, listDockWidget);
		listDockWidget->show();
	}
	// UAS STATUS
	if (infoDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, infoDockWidget);
		infoDockWidget->show();
	}
	// HORIZONTAL SITUATION INDICATOR
	if (hsiDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, hsiDockWidget);
		hsiDockWidget->show();
	}
	// WAYPOINT LIST
	if (waypointDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, waypointDockWidget);
		waypointDockWidget->show();
	}
	// DEBUG CONSOLE
	if (debugConsoleDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, debugConsoleDockWidget);
		debugConsoleDockWidget->show();
	}
	// ONBOARD PARAMETERS
	if (parameterDockWidget)
	{
		addDockWidget(Qt::RightDockWidgetArea, parameterDockWidget);
		parameterDockWidget->show();
	}

	show();
}

void MainWindow::loadPilotView()
{
	clearView();

	// HEAD UP DISPLAY
	if (hudWidget && centerStack)
	{
		centerStack->setCurrentWidget(hudWidget);
		hudWidget->show();
		hudWidget->start();
	}
	if (hddDockWidget1)
	{
		addDockWidget(Qt::RightDockWidgetArea, hddDockWidget1);
		if ( hddDockWidget1->widget() )
		{
			HDDisplay* hddWidget = dynamic_cast<HDDisplay*>( hddDockWidget1->widget() );
			if (hddWidget) hddWidget->start();
		}
		hddDockWidget1->show();
	}
	if (hddDockWidget2)
	{
		addDockWidget(Qt::RightDockWidgetArea, hddDockWidget2);
		if ( hddDockWidget2->widget() )
		{
			HDDisplay* hddWidget = dynamic_cast<HDDisplay*>( hddDockWidget2->widget() );
			if (hddWidget) hddWidget->start();
		}
		hddDockWidget2->show();
	}

	show();
}

void MainWindow::loadOperatorView()
{
	clearView();

	// MAP
	if (mapWidget && centerStack)
	{
		centerStack->setCurrentWidget(mapWidget);
		mapWidget->show();
	}
	// UAS CONTROL
	if (controlDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, controlDockWidget);
		controlDockWidget->show();
	}
	// UAS LIST
	if (listDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, listDockWidget);
		listDockWidget->show();
	}
	// UAS STATUS
	if (infoDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, infoDockWidget);
		infoDockWidget->show();
	}
	// WAYPOINT LIST
	if (waypointDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, waypointDockWidget);
		waypointDockWidget->show();
	}
	// HORIZONTAL SITUATION INDICATOR
	if (hsiDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, hsiDockWidget);
		if ( hsiDockWidget->widget() )
		{
			HSIDisplay* hsiWidget = dynamic_cast<HSIDisplay*>( hsiDockWidget->widget() );
			if (hsiWidget) hsiWidget->start();
		}
		hsiDockWidget->show();
	}
	// OBJECT DETECTION
	if (detectionDockWidget)
	{
		addDockWidget(Qt::RightDockWidgetArea, detectionDockWidget);
		detectionDockWidget->show();
	}
	// PROCESS CONTROL
	if (watchdogControlDockWidget)
	{
		addDockWidget(Qt::RightDockWidgetArea, watchdogControlDockWidget);
		watchdogControlDockWidget->show();
	}

	show();
}

void MainWindow::loadSettingsView()
{
	clearView();

	// LINE CHART
	if (linechartWidget && centerStack)
	{
		linechartWidget->setActive(true);
		centerStack->setCurrentWidget(linechartWidget);
		linechartWidget->show();
	}
    /*
    // COMM XML
    QDockWidget* container1 = new QDockWidget(tr("MAVLink XML to C Code Generator"), this);
    container1->setWidget(protocolWidget);
    addDockWidget(Qt::LeftDockWidgetArea, container1);*/

	// ONBOARD PARAMETERS
	if (parameterDockWidget)
	{
		addDockWidget(Qt::RightDockWidgetArea, parameterDockWidget);
		parameterDockWidget->show();
	}

	show();
}

// Engineer view, used in EMAV2009
void MainWindow::loadEngineerView()
{
	clearView();

	// LINE CHART
	if (linechartWidget && centerStack)
	{
		linechartWidget->setActive(true);
		centerStack->setCurrentWidget(linechartWidget);
		linechartWidget->show();
	}

	// UAS CONTROL
	if (controlDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, controlDockWidget);
		controlDockWidget->show();
	}
	// UAS LIST
	if (listDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, listDockWidget);
		listDockWidget->show();
	}
	// UAS STATUS
	if (infoDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, infoDockWidget);
		infoDockWidget->show();
	}
	// WAYPOINT LIST
	if (waypointDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, waypointDockWidget);
		waypointDockWidget->show();
	}
	// DEBUG CONSOLE
	if (debugConsoleDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, debugConsoleDockWidget);
		debugConsoleDockWidget->show();
	}
	// ONBOARD PARAMETERS
	if (parameterDockWidget)
	{
		addDockWidget(Qt::RightDockWidgetArea, parameterDockWidget);
		parameterDockWidget->show();
	}

	show();
}

void MainWindow::loadMAVLinkView()
{
	clearView();
	if (protocolWidget && centerStack)
	{
		centerStack->setCurrentWidget(protocolWidget);
		protocolWidget->show();
	}
	show();
}

void MainWindow::loadAllView()
{
	clearView();

	// LINE CHART
	if (linechartWidget && centerStack)
	{
		linechartWidget->setActive(true);
		centerStack->setCurrentWidget(linechartWidget);
		linechartWidget->show();
	}
	if (hddDockWidget1)
	{
		addDockWidget(Qt::RightDockWidgetArea, hddDockWidget1);
		if ( hddDockWidget1->widget() )
		{
			HDDisplay* hddWidget = dynamic_cast<HDDisplay*>( hddDockWidget1->widget() );
			if (hddWidget) hddWidget->start();
		}
		hddDockWidget1->show();
	}
	if (hddDockWidget2)
	{
		addDockWidget(Qt::RightDockWidgetArea, hddDockWidget2);
		if ( hddDockWidget2->widget() )
		{
			HDDisplay* hddWidget = dynamic_cast<HDDisplay*>( hddDockWidget2->widget() );
			if (hddWidget) hddWidget->start();
		}
		hddDockWidget2->show();
	}
	// UAS CONTROL
	if (controlDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, controlDockWidget);
		controlDockWidget->show();
	}
	// UAS LIST
	if (listDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, listDockWidget);
		listDockWidget->show();
	}
	// UAS STATUS
	if (infoDockWidget)
	{
		addDockWidget(Qt::LeftDockWidgetArea, infoDockWidget);
		infoDockWidget->show();
	}
	// WAYPOINT LIST
	if (waypointDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, waypointDockWidget);
		waypointDockWidget->show();
	}
	// DEBUG CONSOLE
	if (debugConsoleDockWidget)
	{
		addDockWidget(Qt::BottomDockWidgetArea, debugConsoleDockWidget);
		debugConsoleDockWidget->show();
	}
	// OBJECT DETECTION
	if (detectionDockWidget)
	{
		addDockWidget(Qt::RightDockWidgetArea, detectionDockWidget);
		detectionDockWidget->show();
	}
	// ONBOARD PARAMETERS
	if (parameterDockWidget)
	{
		addDockWidget(Qt::RightDockWidgetArea, parameterDockWidget);
		parameterDockWidget->show();
	}
	
	show();
}

void MainWindow::loadWidgets()
{
    //loadOperatorView();
    loadEngineerView();
    //loadPilotView();
}
