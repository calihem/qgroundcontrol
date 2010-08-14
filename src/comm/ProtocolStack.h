/*=====================================================================
 
PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
 
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
 *   @brief Manage links and protocols
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *   @author Michael Schulz <coding@calihem.de>
 *
 */

#ifndef _PROTOCOLSTACK_H_
#define _PROTOCOLSTACK_H_

#include <QMap>
#include <QList>
#include <QPair>

#include "LinkInterface.h"
#include "ProtocolInterface.h"

/**
 * The ProtocolStack organizes the link and protocol instances. It can manage
 * arbitrary links and takes care of connecting them as well assigning the correct
 * protocol instance to transport the link data into the application.
 *
 **/
/**
 * @brief Start the link managing component.
 *
 * The link manager keeps track of all communication links and provides the global
 * packet queue. It is the main communication hub
 **/

class ProtocolStack : public QObject {
	Q_OBJECT

	public:
		/// enums for link types
		enum LinkType { SerialLink, UDPLink, MAVLinkSimulationLink, UnknownLink };
		/// enums for protocol types
		enum ProtocolType { MAVLinkProtocol, UnknownProtocol };
		typedef QPair<LinkType, LinkInterface*> TypeLinkPair;
		typedef QPair<ProtocolType, ProtocolInterface*> TypeProtocolPair;

		static ProtocolStack& instance();
		/// Add @param link to ProtocolStack
		int addLink(LinkInterface *link);
		/// Build a new link instance of type @param linkType
		static LinkInterface* buildLink(LinkType linkType);
		/// Remove link with @param linkID from ProtocolStack
		int removeLink(int linkID);
		/// Get link instance with link ID @param linkID
		LinkInterface* getLink(int linkID);
		/// Returns a list of all links
		QList<int> getLinkIDs();
		/// Returns the link type of @param link
		static LinkType getLinkType(const LinkInterface *link);
		/// @sa setupProtocol
		int addProtocol(ProtocolInterface *protocol);
		///
		ProtocolInterface* setupProtocol(ProtocolType protocolType);
		///
		int removeProtocol(ProtocolType protocolType);
		/// @sa setupProtocol
		ProtocolInterface* getProtocol(ProtocolType protocolType);
		/// Returns the protocol type of @param protocol
		static ProtocolType getProtocolType(const ProtocolInterface* protocol);
		/// Bind link with @param linkID to protocol of type @param protocolType
		int registerProtocol(int linkID, ProtocolType protocolType);
		/**
		* @brief Get the protocol for this link ID
		*
		* @param linkID link identifier to search for
		* @return A pointer to the link or NULL if not found
		*/
		ProtocolInterface* getProtocolForLink(int linkID);
		/**               
		 * @brief Get linkIDs for given protocol type
		 *
		 * Returns a list of all link IDs which are connected to the
		 * protocol instance of type @param protocolType
		 * @param protocolType type of protocol to search for
		 * @return list of link IDs
		 */
		QList<int> getLinkIDs(ProtocolType protocolType);

	public slots:
		/// Connect all links
		int connectLinks();
		/// Connect link with ID @param linkID
		bool connectLink(int linkID);
		/// Disconnect all links
		int disconnectLinks();
		/// Disconnect link with ID @param linkID
		bool disconnectLink(int linkID);

	private:
		/**
		 * @brief Private singleton constructor
		 *
		 * This class implements the singleton design pattern and has
		 * therefore only a private constructor.
		 */
		ProtocolStack() {};
		~ProtocolStack();
		ProtocolStack(const ProtocolStack &); // intentionally undefined
		ProtocolStack& operator=(const ProtocolStack &); // intentionally undefined

		/// Maps the linkID to the link instance
		QMap<int, LinkInterface*> linkMap; //FIXME: use QVector for faster access
		/// Maps the protocol type to the protocol instance
		QMap<ProtocolType, ProtocolInterface*> protocolMap;
		///
		QMap<LinkInterface*, ProtocolInterface*> linkProtocolMap;

	signals:
		void linkAdded(int linkID);
		void linkRemoved(int linkID);
		void protocolAdded(ProtocolType protocolType);
		void protocolRemoved(ProtocolType protocolType);
};

// ----------------------------------------------------------------------------
// Inline Implementations
// ----------------------------------------------------------------------------
inline LinkInterface* ProtocolStack::getLink(int linkID)
{
	return linkMap.value(linkID, NULL);
}
inline QList<int> ProtocolStack::getLinkIDs()
{
	return linkMap.keys();
}

inline bool ProtocolStack::connectLink(int linkID)
{
	LinkInterface *link = getLink(linkID);
	if (!link) return false;

	return link->open();
}

inline bool ProtocolStack::disconnectLink(int linkID)
{
	LinkInterface *link = getLink(linkID);
	if (!link) return false;

	return link->close();
}

inline ProtocolInterface* ProtocolStack::getProtocol(ProtocolType protocolType)
{
	return protocolMap.value(protocolType, NULL);
}

inline ProtocolInterface* ProtocolStack::getProtocolForLink(int linkID)
{
	LinkInterface *link = getLink(linkID);
	if (!link) return NULL;
	
	return linkProtocolMap.value(link, NULL);
}

#endif // _PROTOCOLSTACK_H_
