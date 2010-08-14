/*=====================================================================
 
PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
 
(c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
 
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
 *   @brief Brief Description
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *   @author Michael Schulz <coding@calihem.de>
 *
 */
#include "ProtocolStack.h"

#include <QApplication>
#include <QDebug>

#include "SerialLink.h"
#include "UDPLink.h"
#include "MAVLinkSimulationLink.h"
#include "MAVLinkProtocol.h"

ProtocolStack& ProtocolStack::instance()
{
	// instantiated on first use and will be guaranteed destroyed 
	static ProtocolStack instance;

	return instance;
}

ProtocolStack::~ProtocolStack()
{
	// delete protocols
	QMap<ProtocolType, ProtocolInterface*>::iterator protocolIterator = protocolMap.begin();
	while ( protocolIterator != protocolMap.end() )
	{
		emit protocolRemoved( protocolIterator.key() );
		if ( protocolIterator.value() ) delete protocolIterator.value();
		protocolIterator = protocolMap.erase(protocolIterator);
	}

	// delete links
	QMap<int, LinkInterface*>::iterator linkIterator = linkMap.begin();
	while ( linkIterator != linkMap.end() )
	{
		emit linkRemoved( linkIterator.key() );
		if ( linkIterator.value() ) delete linkIterator.value();
		linkIterator = linkMap.erase(linkIterator);
	}
}

int ProtocolStack::addLink(LinkInterface *link)
{
	if (!link) return -1;

	linkMap.insert( link->getID(), link);
	emit linkAdded( link->getID() );

	return link->getID();
}

LinkInterface* ProtocolStack::buildLink(LinkType linkType)
{
	if (linkType == SerialLink) return new ::SerialLink();
	if (linkType == UDPLink) return new ::UDPLink();
	if (linkType == MAVLinkSimulationLink) return new ::MAVLinkSimulationLink(":/demo-log.txt");

	return NULL;
}

int ProtocolStack::removeLink(int linkID)
{
	QMap<int, LinkInterface*>::iterator i = linkMap.find(linkID);
	if ( i == linkMap.end() ) //linkID not found
	{
		return 1;
	}
	LinkInterface *link = i.value();

	linkProtocolMap.remove(link);
	linkMap.erase(i);

	emit linkRemoved(linkID);
	delete link;

	return 0;
}

ProtocolStack::LinkType ProtocolStack::getLinkType(const LinkInterface *link)
{
	const ::SerialLink *serialLink = dynamic_cast<const ::SerialLink*>(link);
	if (serialLink) return SerialLink;

	const ::UDPLink *udpLink = dynamic_cast<const ::UDPLink*>(link);
	if (udpLink) return UDPLink;

	const ::MAVLinkSimulationLink *simLink = dynamic_cast<const ::MAVLinkSimulationLink*>(link);
	if (simLink) return MAVLinkSimulationLink;

	return UnknownLink;
}

int ProtocolStack::connectLinks()
{
	int connectionCounter = 0;

	foreach (LinkInterface* link, linkMap)
	{
		if ( link->open() ) connectionCounter++;
	}
	
	return connectionCounter;
}

int ProtocolStack::disconnectLinks()
{
	int disconnectionCounter = 0;

	foreach (LinkInterface* link, linkMap)
	{
		if ( link->close() ) disconnectionCounter++;
	}
	
	return disconnectionCounter;
}

int ProtocolStack::addProtocol(ProtocolInterface *protocol)
{
	if (!protocol) return -1;

	ProtocolType protocolType = getProtocolType(protocol);	
	if (protocolType == UnknownProtocol)
		return 1;
	
	protocolMap.insert(protocolType, protocol);
	emit protocolAdded(protocolType);

	return 0;
}

int ProtocolStack::removeProtocol(ProtocolType protocolType)
{
	ProtocolInterface *protocol = getProtocol(protocolType);
	if (!protocol) return -1;

	protocolMap.remove(protocolType);
	
	emit protocolRemoved(protocolType);
	delete protocol;

	return 0;
}

ProtocolInterface* ProtocolStack::setupProtocol(ProtocolType protocolType)
{
	ProtocolInterface *protocol = getProtocol(protocolType);
	if (protocol) return protocol;
	
	//no protocol of this type available, create new instance
	switch (protocolType)
	{
		case ProtocolStack::MAVLinkProtocol:
			protocol = new ::MAVLinkProtocol();
			protocolMap.insert(protocolType, protocol);
			emit protocolAdded(protocolType);
			return protocol;
		default:
			break;
	}

	return NULL;
}

ProtocolStack::ProtocolType ProtocolStack::getProtocolType(const ProtocolInterface* protocol)
{
	ProtocolType protocolType = UnknownProtocol;
	const ::MAVLinkProtocol *mavProtocol = dynamic_cast<const ::MAVLinkProtocol* >(protocol);
	if (mavProtocol)
		protocolType = MAVLinkProtocol;

	return protocolType;
}

int ProtocolStack::registerProtocol(int linkID, ProtocolType protocolType)
{
	LinkInterface *link = getLink(linkID);
	if (!link) return -1;
	
	ProtocolInterface *protocol = setupProtocol(protocolType);
	if (!protocol) return -2;

	linkProtocolMap.insert(link, protocol);

	connect( link, SIGNAL(bytesReady(LinkInterface*)),
		protocol, SLOT(receiveBytes(LinkInterface*)) );
		
	return 0;
}

QList<int> ProtocolStack::getLinkIDs(ProtocolType protocolType)
{
	//FIXME: improve performance
	ProtocolInterface *protocol = protocolMap.value(protocolType);
	QList<int> linkList;

	QMap<LinkInterface*, ProtocolInterface*>::iterator i;
	for (i = linkProtocolMap.begin(); i != linkProtocolMap.end(); ++i)
	{
		if (i.value() == protocol && i.key() )
			linkList.append( i.key()->getID() );
	}

	return linkList;
}
