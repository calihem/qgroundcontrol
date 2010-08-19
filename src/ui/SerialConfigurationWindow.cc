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
 *   @brief Implementation of configuration window for serial links
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *   @author Michael Schulz <coding@calihem.de>
 *
 */

#include <QDebug>

#include <SerialConfigurationWindow.h>
#include <SerialLinkInterface.h>
#include <qextserialenumerator.h>
#include <QDir>
#if defined (__APPLE__) && defined (__MACH__)
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <paths.h>
#include <termios.h>
#include <sysexits.h>
#include <sys/param.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <AvailabilityMacros.h>

#ifdef __MWERKS__
#define __CF_USE_FRAMEWORK_INCLUDES__
#endif // __MWERKS__

#include <CoreFoundation/CoreFoundation.h>

#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#if defined(MAC_OS_X_VERSION_10_3) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_3)
#include <IOKit/serial/ioss.h>
#endif
#include <IOKit/IOBSD.h>

// Apple internal modems default to local echo being on. If your modem has local echo disabled,
// undefine the following macro.
#define LOCAL_ECHO

#define kATCommandString  "AT\r"

#ifdef LOCAL_ECHO
#define kOKResponseString  "AT\r\r\nOK\r\n"
#else
#define kOKResponseString  "\r\nOK\r\n"
#endif
#endif


// Some helper functions for serial port enumeration
#if defined (__APPLE__) && defined (__MACH__)

enum {
    kNumRetries = 3
              };

// Function prototypes
static kern_return_t FindModems(io_iterator_t *matchingServices);
static kern_return_t GetModemPath(io_iterator_t serialPortIterator, char *bsdPath, CFIndex maxPathSize);

// Returns an iterator across all known modems. Caller is responsible for
// releasing the iterator when iteration is complete.
static kern_return_t FindModems(io_iterator_t *matchingServices)
{
    kern_return_t      kernResult;
    CFMutableDictionaryRef  classesToMatch;

    /*! @function IOServiceMatching
    @abstract Create a matching dictionary that specifies an IOService class match.
    @discussion A very common matching criteria for IOService is based on its class. IOServiceMatching will create a matching dictionary that specifies any IOService of a class, or its subclasses. The class is specified by C-string name.
    @param name The class name, as a const C-string. Class matching is successful on IOService's of this class or any subclass.
    @result The matching dictionary created, is returned on success, or zero on failure. The dictionary is commonly passed to IOServiceGetMatchingServices or IOServiceAddNotification which will consume a reference, otherwise it should be released with CFRelease by the caller. */

    // Serial devices are instances of class IOSerialBSDClient
    classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);
    if (classesToMatch == NULL)
    {
        printf("IOServiceMatching returned a NULL dictionary.\n");
    }
    else {
        /*!
  @function CFDictionarySetValue
  Sets the value of the key in the dictionary.
  @param theDict The dictionary to which the value is to be set. If this
    parameter is not a valid mutable CFDictionary, the behavior is
    undefined. If the dictionary is a fixed-capacity dictionary and
    it is full before this operation, and the key does not exist in
    the dictionary, the behavior is undefined.
  @param key The key of the value to set into the dictionary. If a key
    which matches this key is already present in the dictionary, only
    the value is changed ("add if absent, replace if present"). If
    no key matches the given key, the key-value pair is added to the
    dictionary. If added, the key is retained by the dictionary,
    using the retain callback provided
    when the dictionary was created. If the key is not of the sort
    expected by the key retain callback, the behavior is undefined.
  @param value The value to add to or replace into the dictionary. The value
    is retained by the dictionary using the retain callback provided
    when the dictionary was created, and the previous value if any is
    released. If the value is not of the sort expected by the
    retain or release callbacks, the behavior is undefined.
*/
        CFDictionarySetValue(classesToMatch,
                             CFSTR(kIOSerialBSDTypeKey),
                             CFSTR(kIOSerialBSDModemType));

        // Each serial device object has a property with key
        // kIOSerialBSDTypeKey and a value that is one of kIOSerialBSDAllTypes,
        // kIOSerialBSDModemType, or kIOSerialBSDRS232Type. You can experiment with the
        // matching by changing the last parameter in the above call to CFDictionarySetValue.

        // As shipped, this sample is only interested in modems,
        // so add this property to the CFDictionary we're matching on.
        // This will find devices that advertise themselves as modems,
        // such as built-in and USB modems. However, this match won't find serial modems.
    }

    /*! @function IOServiceGetMatchingServices
        @abstract Look up registered IOService objects that match a matching dictionary.
        @discussion This is the preferred method of finding IOService objects currently registered by IOKit. IOServiceAddNotification can also supply this information and install a notification of new IOServices. The matching information used in the matching dictionary may vary depending on the class of service being looked up.
        @param masterPort The master port obtained from IOMasterPort().
        @param matching A CF dictionary containing matching information, of which one reference is consumed by this function. IOKitLib can contruct matching dictionaries for common criteria with helper functions such as IOServiceMatching, IOOpenFirmwarePathMatching.
        @param existing An iterator handle is returned on success, and should be released by the caller when the iteration is finished.
        @result A kern_return_t error code. */

    kernResult = IOServiceGetMatchingServices(kIOMasterPortDefault, classesToMatch, matchingServices);
    if (KERN_SUCCESS != kernResult)
    {
        printf("IOServiceGetMatchingServices returned %d\n", kernResult);
        goto exit;
    }

    exit:
    return kernResult;
}

/** Given an iterator across a set of modems, return the BSD path to the first one.
 *  If no modems are found the path name is set to an empty string.
 */
static kern_return_t GetModemPath(io_iterator_t serialPortIterator, char *bsdPath, CFIndex maxPathSize)
{
    io_object_t    modemService;
    kern_return_t  kernResult = KERN_FAILURE;
    Boolean      modemFound = false;

    // Initialize the returned path
    *bsdPath = '\0';

    // Iterate across all modems found. In this example, we bail after finding the first modem.

    while ((modemService = IOIteratorNext(serialPortIterator)) && !modemFound)
    {
        CFTypeRef  bsdPathAsCFString;

        // Get the callout device's path (/dev/cu.xxxxx). The callout device should almost always be
        // used: the dialin device (/dev/tty.xxxxx) would be used when monitoring a serial port for
        // incoming calls, e.g. a fax listener.

        bsdPathAsCFString = IORegistryEntryCreateCFProperty(modemService,
                                                            CFSTR(kIOCalloutDeviceKey),
                                                            kCFAllocatorDefault,
                                                            0);
        if (bsdPathAsCFString)
        {
            Boolean result;

            // Convert the path from a CFString to a C (NUL-terminated) string for use
            // with the POSIX open() call.

            result = CFStringGetCString((CFStringRef)bsdPathAsCFString,
                                        bsdPath,
                                        maxPathSize,
                                        kCFStringEncodingUTF8);
            CFRelease(bsdPathAsCFString);

            if (result)
            {
                //printf("Modem found with BSD path: %s", bsdPath);
                modemFound = true;
                kernResult = KERN_SUCCESS;
            }
        }

        printf("\n");

        // Release the io_service_t now that we are done with it.

        (void) IOObjectRelease(modemService);
    }

    return kernResult;
}
#endif

SerialConfigurationWindow::SerialConfigurationWindow(SerialLinkInterface* link, QWidget *parent, Qt::WindowFlags flags) :
	QWidget(parent, flags),
	serialLink(link)
{
	if (!serialLink)
	{
		qDebug() << "Link is NOT a serial link, can't open configuration window";
		return;
	}

	ui.setupUi(this);
	ui.portNameComboBox->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);
	ui.baudRateComboBox->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);

	fillPortNameComboBox();
	fillBaudRateComboBox();

	loadSettingsFromPort();

	portCheckTimer.setInterval(1000);

	setupSignals();

	// Display the widget
	show();
}

SerialConfigurationWindow::~SerialConfigurationWindow() {

}

void SerialConfigurationWindow::fillPortNameComboBox()
{
	ui.portNameComboBox->clear();

#ifdef _TTY_WIN
	QextPortInfo portInfo;
	QList<QextPortInfo> portList = QextSerialEnumerator::getPorts();
	foreach (portInfo, portList)
	{
		ui.portNameComboBox->addItem( portInfo.physName );
	}
#elif defined Q_OS_LINUX
	// TODO Linux has no standard way of enumerating serial ports
	// However the device files are only present when the physical
	// device is connected, therefore listing the files should be
	// sufficient.

	QDir devdir = QDir("/dev");
	QStringList filters;
	filters << "rfcomm*" << "ttyUSB*" << "ttyS*";

	//apply filter on directory
	devdir.setNameFilters(filters);
	devdir.setFilter( QDir::System | QDir::CaseSensitive );
	QStringList entries = devdir.entryList();

	QString entry;
	foreach (entry, entries)
	{
		ui.portNameComboBox->addItem(devdir.absolutePath() + "/" + entry);
	}
#elif defined (__APPLE__) && defined (__MACH__)
	// Enumerate serial ports
	//int            fileDescriptor;
	kern_return_t    kernResult; // on PowerPC this is an int (4 bytes)

	io_iterator_t    serialPortIterator;
	char        bsdPath[MAXPATHLEN];

	kernResult = FindModems(&serialPortIterator);

	kernResult = GetModemPath(serialPortIterator, bsdPath, sizeof(bsdPath));

	IOObjectRelease(serialPortIterator);    // Release the iterator.

	// Add found modems
	if (bsdPath[0])
	{
		if (ui.portNameComboBox->findText(QString(bsdPath)) == -1)
		{
			ui.portNameComboBox->addItem(QString(bsdPath));
			if (!userConfigured) ui.portNameComboBox->setEditText(QString(bsdPath));
		}
	}

	// Add USB serial port adapters
	// TODO Strangely usb serial port adapters are not enumerated, even when connected
	QString devdir = "/dev";
	QDir dir(devdir);
	dir.setFilter(QDir::System);

	QFileInfoList list = dir.entryInfoList();
	for (int i = list.size() - 1; i >= 0; i--)
	{
		QFileInfo fileInfo = list.at(i);
		if (fileInfo.fileName().contains(QString("ttyUSB")) || fileInfo.fileName().contains(QString("ttyS")) || fileInfo.fileName().contains(QString("tty.usbserial")))
		{
			if (ui.portNameComboBox->findText(fileInfo.canonicalFilePath()) == -1)
			{
				ui.portNameComboBox->addItem(fileInfo.canonicalFilePath());
				if (!userConfigured) ui.portNameComboBox->setEditText(fileInfo.canonicalFilePath());
			}
		}
	}
#endif // (__APPLE__) && defined (__MACH__)
	if (ui.portNameComboBox->count() == 0)
	{
		qCritical("No serial ports available on system");
	}
}

void SerialConfigurationWindow::fillBaudRateComboBox()
{
	ui.baudRateComboBox->clear();
#ifdef _TTY_POSIX_
	ui.baudRateComboBox->addItem("50", BAUD50);
	ui.baudRateComboBox->addItem("75", BAUD75);
#endif
	ui.baudRateComboBox->addItem("110", BAUD110);
#ifdef _TTY_POSIX_
	ui.baudRateComboBox->addItem("134", BAUD134);
	ui.baudRateComboBox->addItem("150", BAUD150);
	ui.baudRateComboBox->addItem("200", BAUD200);
#endif
	ui.baudRateComboBox->addItem("300", BAUD300);
	ui.baudRateComboBox->addItem("600",BAUD600);
	ui.baudRateComboBox->addItem("1200", BAUD1200);
#ifdef _TTY_POSIX_
	ui.baudRateComboBox->addItem("1800", BAUD1800);
#endif
	ui.baudRateComboBox->addItem("2400", BAUD2400);
	ui.baudRateComboBox->addItem("4800", BAUD4800);
	ui.baudRateComboBox->addItem("9600", BAUD9600);
#ifdef _TTY_WIN_
	ui.baudRateComboBox->addItem("14400", BAUD14400);
#endif
	ui.baudRateComboBox->addItem("19200", BAUD19200);
	ui.baudRateComboBox->addItem("38400", BAUD38400);
#ifdef _TTY_WIN_
	ui.baudRateComboBox->addItem("56000", BAUD56000);
#endif
	ui.baudRateComboBox->addItem("57600", BAUD57600);
#ifdef _TTY_POSIX_
	ui.baudRateComboBox->addItem("76800", BAUD76800);
#endif
	ui.baudRateComboBox->addItem("115200", BAUD115200);
#ifdef _TTY_WIN_
	ui.baudRateComboBox->addItem("128000", BAUD128000);
	ui.baudRateComboBox->addItem("256000", BAUD256000);
#endif
}

void SerialConfigurationWindow::loadSettingsFromPort()
{
	if (!serialLink) return;
	
	// Set port name
	loadPortNameFromPort();
	
	// Set baud rate
	int index = ui.baudRateComboBox->findData( serialLink->getBaudRate() );
	if (index == -1) qDebug("Unsupported baud rate in Serial Configuration Window");
	ui.baudRateComboBox->setCurrentIndex(index);
	
	// Set flow control
	if (serialLink->getFlowControl() != FLOW_OFF)
	{
		enableFlowControl(true);
	}

	// Set parity
	switch ( serialLink->getParity() )
	{
		case PAR_NONE:
			ui.parNone->setChecked(true);
			break;
		case PAR_ODD:
			ui.parOdd->setChecked(true);
			break;
		case PAR_EVEN:
			ui.parEven->setChecked(true);
			break;
		default:
			qDebug() << "Unsupported parity in Serial Configuration Window";
			break;
	}

	// Set data bits
	ui.dataBitsSpinBox->setValue( (int)serialLink->getDataBits() + 5);

	// Set stop bits
	switch( serialLink->getStopBits() )
	{
		case STOP_1:
			ui.stopBitsSpinBox->setValue(1);
			break;
		case STOP_2:
			ui.stopBitsSpinBox->setValue(2);
			break;
		default:
			qDebug() << "Unsupported stop bits in Serial Configuration Window";
			break;
	}
}

void SerialConfigurationWindow::loadPortNameFromPort()
{
	if (!serialLink) return;

	// if necessary add port name of serial link to combobox
	if (serialLink->getPortName() != "")
	{ // port name set
		int index = ui.portNameComboBox->findText( serialLink->getPortName() );
		if (index == -1)
		{ // port name of serial link is no combobox entry
			ui.portNameComboBox->addItem( serialLink->getPortName() );
		}
		// set entry in combobox
		ui.portNameComboBox->setCurrentIndex(index);
	}
	else
	{ // no port name set
		// take first entry in combobox
		serialLink->setPortName( ui.portNameComboBox->itemText(0) );
	}

	// Set window title
	setTitleToPortName( serialLink->getPortName() );
}

void SerialConfigurationWindow::setupSignals()
{
	if (!serialLink) return;
	
	// Make sure that a change in the link name will be reflected in the UI
	connect(serialLink, SIGNAL(nameChanged(const QString&)),
		this, SLOT(setTitleToPortName(const QString&)));
	// Connect the individual user interface inputs
	connect(ui.portNameComboBox, SIGNAL(editTextChanged(const QString&)),
		this, SLOT(setPortName(const QString&)));
	connect(ui.portNameComboBox, SIGNAL(currentIndexChanged(const QString&)),
		this, SLOT(setPortName(const QString&)));
	connect( &portCheckTimer, SIGNAL(timeout()),
		 this, SLOT(refreshPortNameComboBox()) );
	connect(ui.baudRateComboBox, SIGNAL(activated(int)),
		this, SLOT(setBaudRate(int)));
	connect(ui.flowControlCheckBox, SIGNAL(toggled(bool)),
		this, SLOT(enableFlowControl(bool)));
	connect(ui.parNone, SIGNAL(toggled(bool)),
		this, SLOT(setParityNone()));
	connect(ui.parOdd, SIGNAL(toggled(bool)),
		this, SLOT(setParityOdd()));
	connect(ui.parEven, SIGNAL(toggled(bool)),
		this, SLOT(setParityEven()));
	connect(ui.dataBitsSpinBox, SIGNAL(valueChanged(int)),
		this, SLOT(setDataBits(int)));
	connect(ui.stopBitsSpinBox, SIGNAL(valueChanged(int)),
		this, SLOT(setStopBits(int)));
}

void SerialConfigurationWindow::refreshPortNameComboBox()
{
	ui.portNameComboBox->disconnect();
	fillPortNameComboBox();
	loadPortNameFromPort();
	connect(ui.portNameComboBox, SIGNAL(editTextChanged(const QString&)),
		this, SLOT(setPortName(const QString&)));
	connect(ui.portNameComboBox, SIGNAL(currentIndexChanged(const QString&)),
		this, SLOT(setPortName(const QString&)));
}

void SerialConfigurationWindow::setPortName(const QString& portname)
{
	if (!serialLink) return;

	QString newPort(portname);
#ifdef _WIN32
	newPort = newPort.split("-").first();
#endif
	newPort = newPort.remove(" ");

	if (serialLink->getPortName() != newPort)
	{
		serialLink->setPortName(newPort);
	}
}

void SerialConfigurationWindow::setTitleToPortName(const QString& portname)
{
	setWindowTitle(tr("Configuration of ") + portname);
}

void SerialConfigurationWindow::setBaudRate(int index)
{
	if (!serialLink) return;
	
	serialLink->setBaudRate((BaudRateType)ui.baudRateComboBox->itemData(index).toInt());
}

void SerialConfigurationWindow::enableFlowControl(bool flow)
{
	if (!serialLink) return;

	if(flow)
		serialLink->setFlowControl(FLOW_HARDWARE);
	else
		serialLink->setFlowControl(FLOW_OFF);
}

void SerialConfigurationWindow::setParityNone()
{
	if (!serialLink) return;

	serialLink->setParity(PAR_NONE);
}

void SerialConfigurationWindow::setParityOdd()
{
	if (!serialLink) return;

	serialLink->setParity(PAR_ODD);
}

void SerialConfigurationWindow::setParityEven()
{
	if (!serialLink) return;

	serialLink->setParity(PAR_EVEN);
}

void SerialConfigurationWindow::setDataBits(int bits)
{
	if (!serialLink) return;
	if (bits < 5 || bits > 8) return;

	serialLink->setDataBits( DataBitsType(bits-5) );

}

void SerialConfigurationWindow::setStopBits(int bits)
{
	if (!serialLink) return;
	
	switch(bits)
	{
		case 1:
			serialLink->setStopBits(STOP_1);
			break;
		case 2:
			serialLink->setStopBits(STOP_2);
			break;
		default:
			break;
	}
}

void SerialConfigurationWindow::showEvent(QShowEvent* event)
{
	if (event->isAccepted())
	{
		portCheckTimer.start();
	}
}

void SerialConfigurationWindow::hideEvent(QHideEvent* event)
{
	if (event->isAccepted())
	{
		portCheckTimer.stop();
	}
}
