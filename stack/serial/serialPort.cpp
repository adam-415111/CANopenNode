//#include "StdAfx.h"

//#include <Setupapi.h>
//#pragma comment(lib, "Setupapi.lib")

#include "serialPort.h"
#include <termios.h>    // POSIX terminal control definitions

SerialPort::SerialPort(void) : end_of_line_char_('\n')
{
}

SerialPort::~SerialPort(void)
{
    //stop();
}

char SerialPort::end_of_line_char() const
{
    return this->end_of_line_char_;
}

void SerialPort::end_of_line_char(const char &c)
{
    this->end_of_line_char_ = c;
}

/*std::vector<std::string> SerialPort::get_port_names()
{
    std::vector<std::string> names;

    bool rv;
    DWORD size;
    GUID guid[1];
    HDEVINFO hdevinfo;
    DWORD idx = 0;
    SP_DEVINFO_DATA devinfo_data;
    devinfo_data.cbSize = sizeof(SP_DEVINFO_DATA);
    int count = 0;

    rv = SetupDiClassGuidsFromName("Ports", (LPGUID)&guid, 1, &size) ;
    if (!rv) {
        std::cout << "error : SetupDiClassGuidsFromName() failed..." << std::endl;
        return names;
    }

    hdevinfo = SetupDiGetClassDevs(&guid[0], NULL, NULL, DIGCF_PRESENT | DIGCF_PROFILE);
    if (hdevinfo == INVALID_HANDLE_VALUE) {
        std::cout << "error : SetupDiGetClassDevs() failed..." << std::endl;
        return names;
    }

    while(SetupDiEnumDeviceInfo(hdevinfo, idx++, &devinfo_data)) {
        char friendly_name[MAX_PATH];
        char port_name[MAX_PATH];
        DWORD prop_type;
        DWORD type = REG_SZ;
        HKEY hKey = NULL;

        rv = ::SetupDiGetDeviceRegistryProperty(hdevinfo, &devinfo_data, SPDRP_FRIENDLYNAME, &prop_type,
                                                (LPBYTE)friendly_name, sizeof(friendly_name), &size);
        if (!rv) {
            std::cout << "error : SetupDiGetDeviceRegistryProperty() failed..." << std::endl;
            continue;
        }

        hKey = ::SetupDiOpenDevRegKey(hdevinfo, &devinfo_data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (!hKey) continue;

        size = sizeof(port_name);
        rv = ::RegQueryValueEx(hKey, "PortName", 0, &type, (LPBYTE)&port_name, &size);
        ::RegCloseKey(hKey);

        names.push_back(port_name);
    }

    SetupDiDestroyDeviceInfoList(hdevinfo);

    return names;
}

int SerialPort::get_port_number()
{
    std::vector<std::string> names = get_port_names();
    return names.size();
}

std::string SerialPort::get_port_name(const unsigned int &idx)
{
    std::vector<std::string> names = get_port_names();
    if (idx >= names.size()) return std::string();
    return names[idx];
}

void SerialPort::print_devices()
{
    std::cout << "SerialPort::print_devices()" << std::endl;
    int n = SerialPort::get_port_number();
    for (int i = 0; i < n; ++i) {
        std::string name = SerialPort::get_port_name(i);
        std::cout << "\t" << name.c_str() << std::endl;
    }
}*/

bool SerialPort::start(const char *com_port_name, int baud_rate)
{
    boost::system::error_code ec;

    if (port_) {
        std::cout << "error : port is already opened..." << std::endl;
        return false;
    }

    port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));

    port_->open(com_port_name, ec);
    if (ec) {
        std::cout << "error : port_->open() failed...com_port_name="
                  << com_port_name << ", e=" << ec.message().c_str() << std::endl;
        return false;
    }

    // option settings...
    /*
    port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    port_->set_option(boost::asio::serial_port_base::character_size(8));
    port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    */


    async_read_some_();

    boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));
    return true;
}

void SerialPort::stop()
{
    //std::cout << "stop SerialPort" << std::endl;

    //std::cout << "look" << std::endl;
    boost::mutex::scoped_lock look(mutex_);
    //std::cout << "mutex_" << std::endl;

    //if (port_) {
        //try {
    if (port_->is_open()) {
	port_->cancel();
    }
        //} catch (boost::system::system_error e) {
        //    std::cout << "error cancel r/w's" << ", e=" << e.what() << std::endl;
        //}

        //try {
    port_->close();
        //} catch (boost::system::system_error e) {
        //    std::cout << "error close r/w's" << ", e=" << e.what() << std::endl;
        //}
    //port_.reset();
    //std::cout << "serialPort done" << std::endl;

    io_service_.stop();
    io_service_.reset();
	//std::cout << "io_service_ done" << std::endl;
    //}


}

int SerialPort::write_some(const std::string &buf)
{
    return write_some(buf.c_str(), buf.size());
}

int SerialPort::write_some(const char *buf, const int &size)
{

    if (!port_ || port_.get() == NULL || !port_->is_open()) return -1;
    if (size == 0) return 0;

    boost::mutex::scoped_lock look(mutex_);
    boost::system::error_code ec;

    //std::cout << "look" << "write_some" << std::endl;
    //std::cout << "mutex_" << "write_some" << std::endl;
    int res = port_->write_some(boost::asio::buffer(buf, size), ec);
    //std::cout << "write ec: " << ec.message() << std::endl;

    return res;
}

void SerialPort::async_read_some_()
{
    /*std::cout << "async_read_some_()" << std::endl;
    std::cout << "port_.get(): " << port_.get() << std::endl;
    std::cout << "port_->is_open(): " << port_->is_open() << std::endl;
    std::cout << "io_service_: " << io_service_->stopped() << std::endl;*/
    if (port_ == NULL || port_.get() == NULL || !port_->is_open()) return;
    if (io_service_.stopped()) return;
    //std::cout << "port_->async_read_some!!!" << std::endl;
    port_->async_read_some(
                //boost::asio::async_read(*port_,
                boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
                //receive_buffer,
                //boost::asio::transfer_at_least(29),
                //boost::asio::transfer_at_least(1),
                boost::bind(&SerialPort::on_receive_, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
    //std::cout << "async_read_some_() done" << std::endl;
}

/*void SerialPort::async_read_some_(boost::function<void(const std::string &)> func)
{
    //std::cout << "async_read_some_(boost::function<void(const std::string &)> func)" << std::endl;
    _func = func;
    async_read_some_();
}*/

void SerialPort::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
{
    /* std::cout << "on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)" << std::endl;
    std::cout << "Tbytes_transferred: " << bytes_transferred << std::endl;
    std::cout << "ec: " << ec.message() << std::endl;*/
    boost::mutex::scoped_lock look(mutex_);

    if (port_ == NULL || port_.get() == NULL || !port_->is_open()) return;
    if (io_service_.stopped()) return;
    //std::cout << "bytes_transferred: " << bytes_transferred << std::endl;
    //std::cout << "read_buf_str_: " << read_buf_str_.size() << std::endl;
    /*std::istream input(&receive_buffer);
    std::string line;
    getline(input, line);
    std::cout << "line: " << line.size() << std::endl;
    std::cout << "line: " << std::endl;*/
    //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    if (ec) {
        //std::cout << "on_receive_ ec: " << ec.message() << std::endl;
        if (ec == boost::asio::error::eof && bytes_transferred == 0) {
            //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            async_read_some_();

        }
        return;
    }

    //std::cout << "look" << "on_receive_" << std::endl;

    //std::cout << "mutex_" << "on_receive_" << std::endl;

    for (unsigned int i = 0; i < bytes_transferred; ++i) {
        char c = read_buf_raw_[i];
        //char c = line[i];
        if (c == end_of_line_char_ ) {
            this->_func(read_buf_str_);
            read_buf_str_.clear();
        }
        else {
            read_buf_str_ += c;
            //std::cout << "Unfinished: " << read_buf_str_ << std::endl;
        }
    }
    async_read_some_();
}

/*void SerialPort::on_receive_(const std::string &data)
{
    std::cout << "SerialPort::on_receive_() : " << data << std::endl;
}*/

