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
 *
 */

#ifndef _LINKINTERFACE_H_
#define _LINKINTERFACE_H_

#include <QThread>

/**
 * The link interface defines the interface for all links used to communicate
 * with the groundstation application.
 *
 **/
class LinkInterface : public QThread {
	Q_OBJECT
public:
	LinkInterface(int id = -1);
	virtual ~LinkInterface() {};

	/* Connection management */

	/**
	 * @brief Get the ID of this link
	 *
	 * The ID is an (unique) integer for accessing a link. It similar to
	 * file descriptors in POSIX operating systems. If correctly set it
	 * starts at 0.
	 *
	 * @return ID of this link
	 * @see ProtocolStack
	 */
	virtual int getID() const;

	/**
	 * @brief Set the ID of this link
	 */
	virtual void setID(int id);

	/**
	 * @brief Get the human readable name of this link
	 */
	virtual const QString& getName() const;

	/**
	 * @brief Set the human readable name of this link
	 */
	virtual void setName(const QString& name);

	/**
	 * @brief Determine the connection status
	 *
	 * @return True if the connection is established, false otherwise
	 **/
	virtual bool isConnected() const = 0;

	/* Connection characteristics */

	/**
	 * @Brief Get the nominal data rate of the interface.
	 *
	 * The nominal data rate is the theoretical maximum data rate of the
	 * interface. For 100Base-T Ethernet this would be 100 Mbit/s (100'000'000
	 * Bit/s, NOT 104'857'600 Bit/s).
	 *
	 * @return The nominal data rate of the interface in bit per second, 0 if unknown
	 * @see getLongTermDataRate() For the mean data rate
	 * @see getShortTermDataRate() For a the mean data rate of the last seconds
	 * @see getCurrentDataRate() For the data rate of the last transferred chunk
	 * @see getMaxDataRate() For the maximum data rate
	 **/
	virtual qint64 getNominalDataRate() const = 0;

	/**
	 * @brief Full duplex support of this interface.
	 *
	 * This method returns true if the interface supports full duplex, which implies
	 * the full datarate when sending and receiving data simultaneously.
	 *
	 * @return True if the interface supports full duplex, false otherwise
	 **/
	virtual bool isFullDuplex() const = 0;

	/**
	 * @brief Get the link quality.
	 *
	 * The link quality is reported as percent, on a scale from 0 to 100% in 1% increments.
	 * If this feature is not supported by the interface, a call to this method return -1.
	 *
	 * @return The link quality in integer percent or -1 if not supported
	 **/
	virtual int getLinkQuality() const = 0;

	/**
	 * @Brief Get the long term (complete) mean of the data rate
	 *
	 * The mean of the total data rate. It is calculated as
	 * all transferred bits / total link uptime.
	 *
	 * @return The mean data rate of the interface in bit per second, 0 if unknown
	 * @see getNominalDataRate() For the nominal data rate of the interface
	 * @see getShortTermDataRate() For a the mean data rate of the last seconds
	 * @see getCurrentDataRate() For the data rate of the last transferred chunk
	 * @see getMaxDataRate() For the maximum data rate
	 **/
	virtual qint64 getTotalUpstream() const = 0;

	/**
	 * @Brief Get the current data rate
	 *
	 * The datarate of the last 100 ms
	 *
	 * @return The mean data rate of the interface in bit per second, 0 if unknown
	 * @see getNominalDataRate() For the nominal data rate of the interface
	 * @see getLongTermDataRate() For the mean data rate
	 * @see getShortTermDataRate() For a the mean data rate of the last seconds
	 * @see getMaxDataRate() For the maximum data rate
	 **/
	virtual qint64 getCurrentUpstream() const = 0;

	/**
	 * @Brief Get the maximum data rate
	 *
	 * The maximum peak data rate.
	 *
	 * @return The mean data rate of the interface in bit per second, 0 if unknown
	 * @see getNominalDataRate() For the nominal data rate of the interface
	 * @see getLongTermDataRate() For the mean data rate
	 * @see getShortTermDataRate() For a the mean data rate of the last seconds
	 * @see getCurrentDataRate() For the data rate of the last transferred chunk
	 **/
	virtual qint64 getMaxUpstream() const = 0;

	/**
	 * @Brief Get the total number of bits sent
	 *
	 * @return The number of sent bits
	 **/
	virtual qint64 getBitsSent() const = 0;

	/**
	 * @Brief Get the total number of bits received
	 *
	 * @return The number of received bits
	 * @bug Decide if the bits should be counted fromt the instantiation of the interface or if the counter should reset on disconnect.
	 **/
	virtual qint64 getBitsReceived() const = 0;

	/**
	 * @brief Open (connect) this interface logically
	 *
	 * @return True if connection could be established, false otherwise
	 **/
	virtual bool open() = 0;

	/**
	 * @brief Close (disconnect) this interface logically
	 *
	 * @return True if connection could be terminated, false otherwise
	 **/
	virtual bool close() = 0;

	/**
	 * @brief Get the current number of bytes in buffer.
	 *
	 * @return The number of bytes ready to read
	 **/
        virtual qint64 bytesAvailable() const = 0;

public slots:

	/**
	 * @brief This method allows to write bytes to the interface.
	 *
	 * If the underlying communication is packet oriented,
	 * one write command equals a datagram. In case of serial
	 * communication arbitrary byte lengths can be written
	 *
	 * @param bytes The pointer to the byte array containing the data
	 * @param length The length of the data array
	 * @return The number of written bytes
	 **/
	virtual qint64 writeBytes(const char *bytes, qint64 length) = 0;

	/**
	 * @brief Read a number of bytes from the interface.
	 *
	 * @param bytes The pointer to write the bytes to
	 * @param maxLength The maximum length which can be written
	 * @return The number of readed bytes
	 **/
	virtual qint64 readBytes(char *bytes, qint64 maxLength) = 0;

signals:
	/**
	 * @brief This signal is emitted when new data is ready to read in the queue
	 *
	 * @param linkID The ID of the link to read from
	 **/
	void bytesReady(int linkID);

        /**
         * @brief New data arrived
         *
         * The use of this function is discouraged for high-performance links.
         *
         * @param data the new bytes
         */
        void bytesReceived(int linkID, const QByteArray &data);

	/**
	 * @brief This signal is emitted instantly when the link is connected
	 **/
	void opened();

	/**
	 * @brief This signal is emitted instantly when the link is disconnected
	 **/
	void closed();

	/**
	 * @brief This signal is emitted instantly when the link status changes
	 **/
	void opened(bool opened);

        /**
         * @brief This signal is emitted if the human readable name of this link changes
         */
        void nameChanged(const QString &name);

protected:
	int id;
	QString name;

};

// ----------------------------------------------------------------------------
// Inline Implementations
// ----------------------------------------------------------------------------
inline LinkInterface::LinkInterface(int id) :
	id(id)
{
}

inline int LinkInterface::getID() const
{
	return id;
}

inline void LinkInterface::setID(int id)
{
	LinkInterface::id = id;
}

inline const QString& LinkInterface::getName() const
{
	return name;
}

inline void LinkInterface::setName(const QString& name)
{
	LinkInterface::name = name;
	emit nameChanged(LinkInterface::name);
}

/* Declare C++ interface as Qt interface */
//Q_DECLARE_INTERFACE(LinkInterface, "org.openground.comm.links.LinkInterface/1.0")

#endif // _LINKINTERFACE_H_
