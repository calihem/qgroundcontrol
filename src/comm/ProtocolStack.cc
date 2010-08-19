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
#ifdef OPAL_RT
#include "OpalLink.h"
#endif
#include "MAVLinkProtocol.h"

ProtocolStack& ProtocolStack::instance()
{
	// instantiated on first use and will be guaranteed destroyed 
	static ProtocolStack instance;

	return instance;
}

ProtocolStack::ProtocolStack() : linkVector(4, NULL)
{
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
	for (int id = 0; id < linkVector.size(); ++id)
	{
		if ( linkVector.at(id) )
		{
			emit linkRemoved( id );
			delete linkVector.at(id);
		}
	}
}

int ProtocolStack::addLink(LinkInterface *link)
{
	if (!link) return -1;

	bool foundFree = false;
	for (int id = 0; id < linkVector.size(); ++id)
	{ // take first free place in linkVector
		if ( !linkVector.at(id) )
		{
			link->setID(id);
			foundFree = true;
			break;
		}
	}
	if (!foundFree)
	{ // append link to linkVector
		link->setID( linkVector.size() );
	}

	linkVector.insert(link->getID(), link);
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
	LinkInterface *link = getLink(linkID);
	if (!link)
	{ // no link with this ID known
		return 1;
	}

	linkProtocolMap.remove(link);
	linkVector[linkID] = NULL;

	emit linkRemoved(linkID);
	delete link;

	return 0;
}

QList<LinkInterface*> ProtocolStack::getLinks()
{
	//FIXME: improve performance
	QList<LinkInterface*> linkList;
	for (int i=0; i<linkVector.size(); i++)
	{
		if (linkVector.at(i))
		{
			linkList.append(linkVector.at(i));
		}
	}
	return linkList;
}

QList<int> ProtocolStack::getLinkIDs()
{
	//FIXME: improve performance
	QList<int> linkIDList;
	for (int i=0; i<linkVector.size(); i++)
	{
		if (linkVector.at(i))
		{
			linkIDList.append(i);
		}
	}
	return linkIDList;
}

ProtocolStack::LinkType ProtocolStack::getLinkType(const LinkInterface *link)
{
	const ::SerialLink *serialLink = dynamic_cast<const ::SerialLink*>(link);
	if (serialLink) return SerialLink;

	const ::UDPLink *udpLink = dynamic_cast<const ::UDPLink*>(link);
	if (udpLink) return UDPLink;

	const ::MAVLinkSimulationLink *simLink = dynamic_cast<const ::MAVLinkSimulationLink*>(link);
	if (simLink) return MAVLinkSimulationLink;

#ifdef OPAL_RT
	const ::OpalLink* opalLink = dynamic_cast<const ::OpalLink*>(link);
	if (opalLink) return OpalLink;
#endif

	return UnknownLink;
}

int ProtocolStack::connectLinks()
{
	int connectionCounter = 0;

	for (int id = 0; id < linkVector.size(); ++id)
	{
		if ( linkVector.at(id) )
			if ( linkVector.at(id)->open() ) connectionCounter++;
	}

	return connectionCounter;
}

int ProtocolStack::disconnectLinks()
{
	int disconnectionCounter = 0;

	for (int id = 0; id < linkVector.size(); ++id)
	{
		if ( linkVector.at(id) )
			if ( linkVector.at(id)->close() ) disconnectionCounter++;
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

ProtocolInterface* ProtocolStack::addProtocol(ProtocolType protocolType)
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
	
	ProtocolInterface *protocol = addProtocol(protocolType);
	if (!protocol) return -2;

	linkProtocolMap.insert(link, protocol);

	if (link->getReadingMode() == LinkInterface::AutoReading)
	{
		connect( link, SIGNAL(dataReceived(int, const QByteArray&)),
			protocol, SLOT(handleLinkInput(int, const QByteArray&)) );
	}
	else if (link->getReadingMode() == LinkInterface::ManualReading)
	{
		connect( link, SIGNAL(readyRead(int)),
			protocol, SLOT(readFromLink(int)) );
	}

	connect( protocol, SIGNAL(dataToSend(const QByteArray&)),
			link, SLOT(write(const QByteArray&)) );

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

