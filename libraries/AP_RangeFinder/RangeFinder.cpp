Inside detect_instance() function in switch case : 
case RangeFinder_TYPE_IOTECH_Radar:
         if(AP_RangeFinder_IOTECH_RADAR::detect()){
               drivers[instance] = new AP_RangeFinder_IOTECH_RADAR(state[instance], params[instance]);
         }
         break;
