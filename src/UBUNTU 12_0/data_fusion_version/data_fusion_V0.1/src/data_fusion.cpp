#include "data_fusion.h"

data_fusion::data_fusion(std::string Port,int baud_rate,size_t timeout,int Freq):
																		io(),port_server(io,Port),timeout(timeout),timer_read(port_server.get_io_service()),
																		timer_program_freq(io_freq,boost::posix_time::milliseconds(timeout)),read_error(true),
																		Freq_(Freq){
	port_server.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
//	freq_serial_read();
	}
	
void data_fusion::read_complete(const boost::system::error_code& error,size_t bytes_transferred){
	read_error = (error || bytes_transferred == 0);
	timer_read.cancel();
	}
	
void data_fusion::time_out(const boost::system::error_code& error){
	if(error){
		return;
		}
	port_server.cancel();
	}
	
bool data_fusion::read_char(char& val){
	val = c = '\0';
	port_server.get_io_service().reset();
	boost::asio::async_read(port_server,boost::asio::buffer(&c,1),
	boost::bind(&data_fusion::read_complete,this,
				 boost::asio::placeholders::error,
				 boost::asio::placeholders::bytes_transferred
				)
	);
	timer_read.expires_from_now(boost::posix_time::milliseconds(timeout));
	timer_read.async_wait(boost::bind(&data_fusion::time_out,
								 this, boost::asio::placeholders::error));
	port_server.get_io_service().run();
	if(!read_error)
	val = c;
	timer_read.cancel();
	return !read_error;
	}

void data_fusion::serial_read(){

	char C;
	gryo_data_set.clear();
	while(read_char(C)){
		std::string rsp;
		bool start_flag = false;
	    bool end_flag = false;
		if(C == 'S'){
		std::string data_array[10];
		int index = 0;
		start_flag = true;
		gryo_data gryo_data_;
		while(read_char(C)){
			if(C == 'E'){
				end_flag = true;
				break;
			}
			else if(C == ' ') {
				data_array[index] = rsp;
				index ++;	
				rsp= "";				
				}
			else{
				rsp += C;
				}
		}
		if(start_flag && end_flag){
			gryo_data_.time_stamp = data_array[0];
			for(int i = 0;i<3;i++){			
				gryo_data_.angle[i] = data_array[i+1];
				gryo_data_.angle_vel[i] = data_array[i+4];
				gryo_data_.linear_acc[i] = data_array[i+7];			
				} 
			std::cout<<"time: "<<data_array[0]<<"\n";
			std::cout<<"angle: "<<data_array[1]<<" "<<data_array[2]<<" "<<data_array[3]<<"\n";
			std::cout<<"angle_vel: "<<data_array[4]<<" "<<data_array[5]<<" "<<data_array[6]<<"\n";
			std::cout<<"linear_acc: "<<data_array[7]<<" "<<data_array[8]<<" "<<data_array[9]<<"\n";
			gryo_data_set.push_back(gryo_data_);
			}
		else{
			std::cout<< "Lost message";
			}
		
		}
		else{
		 continue;
		 }	
	}
	std::cout<<"data_buffer_size: " <<gryo_data_set.size()<<"\n";

}

void data_fusion::freq_serial_read(){
		io_freq.reset();
		timer_program_freq.expires_from_now(boost::posix_time::milliseconds(int(1/Freq_*1000)));
		timer_program_freq.async_wait(boost::bind(&data_fusion::serial_read,this));
		io_freq.run();
		timer_program_freq.cancel();
	
	}
int main(int argc, char **argv)
{
	std::vector<data_fusion::gryo_data> test;
	data_fusion data_fusion_("/dev/ttyACM0",9600,1,20);
	while(true){
	data_fusion_.freq_serial_read();
	}

 return 0;
}

	
