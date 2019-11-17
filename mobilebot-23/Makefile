all:
	@make -C lcmtypes --no-print-directory                                                        
	@make -C mobilebot --no-print-directory
	@make -C test_motors --no-print-directory
	@make -C measure_motors --no-print-directory
	@make -C optitrack/common --no-print-directory                                                
	@make -C optitrack --no-print-directory
	@make -C optitrack_client --no-print-directory
	@make -C python                                                   

laptop:	
	#source setenv.sh
	@make -C lcmtypes --no-print-directory
	@make -C optitrack/common --no-print-directory                                                
	@make -C optitrack --no-print-directory    
	@make -C optitrack_client --no-print-directory                                                 
	@make -C java --no-print-directory
	@make -C python --no-print-directory

clean:
	@make -C lcmtypes -s clean                                                 
	@make -C mobilebot -s clean
	@make -C test_motors -s clean
	@make -C measure_motors -s clean
	@make -C optitrack/common -s clean                                         
	@make -C optitrack -s clean
	@make -C optitrack_client -s clean                                               
	@make -C java -s clean
	@make -C python -s clean
