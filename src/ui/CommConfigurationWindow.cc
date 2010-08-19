/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Implementation of configuration window for communication links
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *   @author Michael Schulz <coding@calihem.de>
 *
 *   @sa CommConfigurationWindow.h
 */

#include <QDebug>

#include <QBoxLayout>
#include <QWidget>

#include "CommConfigurationWindow.h"
#include "SerialLink.h"
#include "SerialConfigurationWindow.h"
#include "UDPLink.h"
#include "MAVLinkSimulationLink.h"
#ifdef OPAL_RT
#include "OpalLink.h"
#endif
#include "MAVLinkProtocol.h"
#include "MAVLinkSettingsWidget.h"

CommConfigurationWindow::CommConfigurationWindow(LinkInterface* link, ProtocolInterface* protocol, QWidget *parent, Qt::WindowFlags flags) :
	QWidget(parent, flags),
	typeLinkPair( ProtocolStack::getLinkType(link), link),
	typeProtocolPair( ProtocolStack::getProtocolType(protocol), protocol)
{
	//set up default UI
	ui.setupUi(this);

	// add needed layouts to groupbox (Is it possible to do it in Qt designer?)
	linkLayout = new QBoxLayout(QBoxLayout::LeftToRight, ui.linkGroupBox);
	protocolLayout = new QBoxLayout(QBoxLayout::LeftToRight, ui.protocolGroupBox);

	// if necessary create new link
	bool newLink = false;
	if (!typeLinkPair.second)
	{ // create new link
		if ( createLink((ProtocolStack::LinkType)ui.linkType->currentIndex()) ) return;
		newLink = true;
	}

	// if necessary set up new protocol
	bool newProtocol = false;
	if (!typeProtocolPair.second)
	{ //create/get protocol instance
		// is there already a protocol registered for this link?
		typeProtocolPair.second = ProtocolStack::instance().getProtocolForLink(typeLinkPair.second->getID());
		if (typeProtocolPair.second)
		{ //protocol already registered
			typeProtocolPair.first = ProtocolStack::getProtocolType(typeProtocolPair.second);
		}
		else
		{ //no protocol registered so far
			typeProtocolPair.first = (ProtocolStack::ProtocolType)ui.protocolType->currentIndex();
			typeProtocolPair.second = ProtocolStack::instance().addProtocol(typeProtocolPair.first);
			newProtocol = true;
		}
	}
	if (!typeProtocolPair.second)
	{ //error
		qDebug("No protocol available");
		// disable UI
		setEnabled(false);
		return;
	}

	// set up UI for link and protocol
	setupUI(typeLinkPair);
	setupUI(typeProtocolPair);

	// set the connection state
	setConnectionState( typeLinkPair.second->isConnected() );

	if (newLink || newProtocol)
	{ // disable delete button
		ui.deleteButton->setEnabled(false);
	}
	else
	{
		// disable comboboxes
		ui.linkType->setEnabled(false);
		ui.protocolType->setEnabled(false);
	}

	setupLinkSignals();
	// set up ProtocolStack signals
	connect( &ProtocolStack::instance(), SIGNAL(linkAdded(int)),
		this, SLOT(setAddedLink(int)) );
	connect( &ProtocolStack::instance(), SIGNAL(linkRemoved(int)),
		this, SLOT(setRemovedLink(int)) );
	// set up user actions
	connect(ui.connectButton, SIGNAL(clicked()), this, SLOT(toggleConnection()));
	connect(ui.deleteButton, SIGNAL(clicked()), this, SLOT(removeLink()));
	connect(ui.closeButton, SIGNAL(clicked()), this->window(), SLOT(close()));
	// set up combobox signals
	connect( ui.linkType, SIGNAL(currentIndexChanged(int)),
		this, SLOT(changeLink(int)) );
	connect( ui.protocolType, SIGNAL(currentIndexChanged(int)),
		this, SLOT(changeProtocol(int)) );

	hide();
}

CommConfigurationWindow::~CommConfigurationWindow()
{
	if (typeLinkPair.second)
		if ( typeLinkPair.second != ProtocolStack::instance().getLink(typeLinkPair.second->getID()) )
		{ //link doesn't belong to ProtocolStack
			delete typeLinkPair.second;
		}
}

void CommConfigurationWindow::setupUI(const ProtocolStack::TypeLinkPair &pair)
{
	if (pair.first == ProtocolStack::UnknownLink)
	{
		qDebug("Unknown link");
		// disable UI
		setEnabled(false);
		return;
	}

	// set combobox
	ui.linkType->setCurrentIndex(pair.first);
	if (linkConfigWidget) delete linkConfigWidget;

	// create new configuration widget
	switch (pair.first)
	{
		case ProtocolStack::SerialLink:
			linkConfigWidget = new SerialConfigurationWindow(dynamic_cast<SerialLinkInterface*>(pair.second), this);
			linkLayout->addWidget(linkConfigWidget);
			ui.linkGroupBox->setTitle( tr("Serial Link") );
			break;
		case ProtocolStack::UDPLink:
			ui.linkGroupBox->setTitle( tr("UDP Link") );
			//TODO
			break;
		case ProtocolStack::MAVLinkSimulationLink:
			ui.linkGroupBox->setTitle( tr("MAVLink Simulation Link") );
			//TODO
			break;
#ifdef OPAL_RT
		case ProtocolStack::OpalLink:
			ui.linkGroupBox->setTitle(tr("Opal-RT Link"));
			//TODO
			break;
#endif
		default:
			ui.linkGroupBox->setTitle( tr("Link") );
			break;
	}

	setTitle( pair.second->getName() );
}

void CommConfigurationWindow::setupUI(const ProtocolStack::TypeProtocolPair &pair)
{
	if (pair.first == ProtocolStack::UnknownProtocol)
	{
		qDebug("Unknown protocol");
		// disable UI
		setEnabled(false);
		return;
	}

	// set combobox
	ui.protocolType->setCurrentIndex(pair.first);

	if (protocolConfigWidget) delete protocolConfigWidget;
	switch (pair.first)
	{
		case ProtocolStack::MAVLinkProtocol:
			protocolConfigWidget =  new MAVLinkSettingsWidget(dynamic_cast<MAVLinkProtocol*>(pair.second), this);
			protocolLayout->addWidget(protocolConfigWidget);
			ui.protocolGroupBox->setTitle( pair.second->getName() );
			break;
		default:
			ui.protocolGroupBox->setTitle( tr("Protocol") );
			return;
	}
}

void CommConfigurationWindow::setupLinkSignals()
{
	connect( typeLinkPair.second, SIGNAL(opened(bool)),
		this, SLOT(setConnectionState(bool)) );
	// make sure that a change in the link name will be reflected in the UI
	connect(typeLinkPair.second, SIGNAL(nameChanged(const QString&)),
		this, SLOT(setTitle(const QString&)));
}

void CommConfigurationWindow::changeLink(int linkIndex)
{
	if (typeLinkPair.second)
		if ( typeLinkPair.second != ProtocolStack::instance().getLink(typeLinkPair.second->getID()) )
		{ //link doesn't belong to ProtocolStack
			delete typeLinkPair.second;
		}
	typeLinkPair.second = NULL;
	typeLinkPair.first = ProtocolStack::UnknownLink;
	setWindowTitle( tr("Communication Configuration Window") );

	if (linkConfigWidget)
	{
		linkLayout->removeWidget(linkConfigWidget);
		delete linkConfigWidget;
	}

	// create new link 
	if ( createLink((ProtocolStack::LinkType)linkIndex) ) return;
	setupUI(typeLinkPair);

	// set up signals of created link
	setupLinkSignals();
}

void CommConfigurationWindow::changeProtocol(int protocolIndex)
{
	typeProtocolPair.second = NULL;
	typeProtocolPair.first = ProtocolStack::UnknownProtocol;

	if (protocolConfigWidget)
	{
		protocolLayout->removeWidget(protocolConfigWidget);
		delete protocolConfigWidget;
	}

	// get new protocol and set up UI
	typeProtocolPair.first = (ProtocolStack::ProtocolType)protocolIndex;
	typeProtocolPair.second = ProtocolStack::instance().addProtocol(typeProtocolPair.first);
	setupUI(typeProtocolPair);
}

void CommConfigurationWindow::addLink()
{
	ProtocolStack::instance().addLink(typeLinkPair.second);
	ProtocolStack::instance().registerProtocol(typeLinkPair.second->getID(), typeProtocolPair.first);
}

void CommConfigurationWindow::setAddedLink(int linkID)
{
	//our link?
	if (typeLinkPair.second->getID() != linkID) return;

	// set connection state
	setConnectionState( typeLinkPair.second->isConnected() );
	// enable delete button
	ui.deleteButton->setEnabled(true);
	// disable comboboxes
	ui.linkType->setEnabled(false);
	ui.protocolType->setEnabled(false);
	// connect
	if ( !typeLinkPair.second->isConnected() )
		typeLinkPair.second->open();
}

void CommConfigurationWindow::removeLink()
{
	ProtocolStack::instance().removeLink( typeLinkPair.second->getID() );
}

void CommConfigurationWindow::setRemovedLink(int linkID)
{
	//our link?
	if (typeLinkPair.second->getID() != linkID) return;
	typeLinkPair.second = NULL;

	// enable comboboxes
	ui.linkType->setEnabled(true);
	ui.protocolType->setEnabled(true);
	// create new link and widget
	changeLink( ui.linkType->currentIndex() );
	// disable delete button
	ui.deleteButton->setEnabled(false);
}


void CommConfigurationWindow::toggleConnection()
{
	if ( typeLinkPair.second->isConnected() )
		typeLinkPair.second->close();
	else
	{
		if (ProtocolStack::instance().getLink(typeLinkPair.second->getID()) )
		{ // link belongs to ProtocolStack
			typeLinkPair.second->open();
		}
		else
		{ // link doesn't belong to ProtocolStack
			addLink();
		}
	}
}

void CommConfigurationWindow::setConnectionState(bool connected)
{
	if(connected)
	{
		ui.connectionStatusLabel->setText( tr("Connected") );
		ui.connectButton->setText( tr("Disconnect") );
		ui.deleteButton->setEnabled(false);
		ui.linkGroupBox->setEnabled(false);
	}
	else
	{
		ui.connectionStatusLabel->setText( tr("Disconnected") );
		ui.connectButton->setText( tr("Connect") );
		ui.deleteButton->setEnabled(true);
		ui.linkGroupBox->setEnabled(true);
	}
}

void CommConfigurationWindow::setTitle(const QString &linkname)
{
	setWindowTitle(tr("Settings for ") + linkname);
}
