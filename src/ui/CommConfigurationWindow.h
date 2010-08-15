/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

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
 *   @brief Definition of configuration window for communication links
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *   @author Michael Schulz <coding@calihem.de>
 *
 *   @sa CommConfigurationWindow.cpp
 */
#ifndef _COMMCONFIGURATIONWINDOW_H_
#define _COMMCONFIGURATIONWINDOW_H_

#include <QObject>
#include <QWidget>
#include <QPointer>
#include <QAction>
#include "ProtocolStack.h"
#include "ui_CommSettings.h"

class CommConfigurationWindow : public QWidget
{
	Q_OBJECT

	public:
		CommConfigurationWindow(LinkInterface* link = 0, ProtocolInterface* protocol = 0, QWidget *parent = 0, Qt::WindowFlags flags = Qt::Sheet);
		~CommConfigurationWindow();

	public slots:

	private:
		Ui::commSettings ui;
		QBoxLayout *linkLayout;
		QBoxLayout *protocolLayout;
		ProtocolStack::TypeLinkPair typeLinkPair;
		ProtocolStack::TypeProtocolPair typeProtocolPair;
		QPointer<QWidget> linkConfigWidget;
		QPointer<QWidget> protocolConfigWidget;

		/// Sets current link to link of type @param linkType
		int createLink(ProtocolStack::LinkType linkType);
		/// Sets up UI for link given by @param pair
		void setupUI(const ProtocolStack::TypeLinkPair &pair);
		/// Sets up UI for protocol given by @param pair
		void setupUI(const ProtocolStack::TypeProtocolPair &pair);
		/// If @param add is true manageButton will be set do add buttton, otherwise to delete button 
		void setupEditButton(bool add);
		/// Connects all necessary signals and slots correspondig to current link 
		void setupLinkSignals();

	private slots:
		/// Add current link to ProtocolStack
		void addLink();
		/// Set up UI after @param link has been added to ProtocolStack
		void setAddedLink(int linkID);
		/// Remove current link from ProtocolStck
		void removeLink();
		/**
		 * @brief Remove link from UI
		 *
		 * Gets called after ProtocolStack has succesfully removed link
		 * with ID @param linkID. The UI will be refreshed to add add a
		 * new link.
		 */
		void setRemovedLink(int linkID);
		/// Change current link to link type given by @param linkIndex 
		void changeLink(int linkIndex);
		/// Change current protocol to protocol type given by @param protocolIndex 
		void changeProtocol(int protocolIndex);
		/// Sets the window title to @param linkname
		void setTitle(const QString &linkname);
		/// Sets the connection state to @param connected
		void setConnectionState(bool connected);
		/// Toggles current link between connected and disconnected
		void toggleConnection();

};

// ----------------------------------------------------------------------------
// Inline Implementations
// ----------------------------------------------------------------------------
inline int CommConfigurationWindow::createLink(ProtocolStack::LinkType linkType)
{
	typeLinkPair.first = linkType;
	typeLinkPair.second = ProtocolStack::buildLink(typeLinkPair.first);
	if (!typeLinkPair.second)
	{
		qDebug("Creation of new link failed");
		setEnabled(false);
		return -1;
	}
	
	return 0;
}

#endif // _COMMCONFIGURATIONWINDOW_H_
