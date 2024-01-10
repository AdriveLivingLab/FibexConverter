import os
import time
import configuration_to_fros
from jinja2 import Environment, FileSystemLoader
import re
import copy

class FileWriter():
    def __init__(self, filename, channel_name):
        self.filename = filename
        self.channel_name = channel_name
        (path, f) = os.path.split(filename)
        filenoext = '.'.join(f.split('.')[:-1])
        output_dir = os.path.join(path, filenoext)
        self.target_dir = os.path.join(output_dir, "")


        self.__content__ = str()
        self.prepare_output_dirs()
    
    def prepare_output_dirs(self):
        
        self.packagefolder = 'ixxat_gw_'+self.channel_name.lower()
        
        folders = ['ROS1', 'ROS2', 
                   'ROS1\\flexray_msgs', 'ROS1\\flexray_msgs\\msg', 'ROS1\\'+self.packagefolder, 'ROS1\\'+self.packagefolder+'\\include', 'ROS1\\'+self.packagefolder+'\\launch', 'ROS1\\'+self.packagefolder+'\\src',
                   'ROS2\\flexray_msgs', 'ROS2\\flexray_msgs\\msg', 'ROS2\\'+self.packagefolder, 'ROS2\\'+self.packagefolder+'\\include', 'ROS2\\'+self.packagefolder+'\\launch', 'ROS2\\'+self.packagefolder+'\\src']
        
        if not os.path.exists(self.target_dir):
            os.makedirs(self.target_dir)
            time.sleep(0.5)
        
        for folder in folders:
            path = os.path.join(self.target_dir, folder)
            if not os.path.exists(path):
                os.mkdir(path)
                time.sleep(0.5)


    def write_content_to_file(self, filename, content):
        file = os.path.join(self.target_dir, filename)
        with open(file, "w+") as f:
            f.write("%s" % content)
            print(f"... wrote {filename}")

    def write_file(self, filename):
        file = os.path.join(self.target_dir, filename)
        with open(file, "w+") as f:
            f.write("%s" % self.__content__)
        self.clear_content()
    
    def copy_file(self, source, target):
        from shutil import copyfile
        try:
            copyfile(source, target)
        except IOError as e:
            print("Unable to copy file. %s Please check file permissions. Is it opened somewhere?" % e)
        except:
            print("Unexpected error")

    def get_from_dict_or_none(self, d, key):
        if d is None:
            return None
        return self.get_from_dict(d, key, None)
    
    @staticmethod
    def get_from_dict(d, key, default):
        if d is None or key is None or key not in d:
            return default
        return d[key]
    
    def get_raw_signal_format(self, length):
        #unsigned_sig_type = ["bool", "uint8", "uint16", "uint32", "uint64"]
        if length == 1:
            return "bool"
        elif length <=8:
            return "uint8_t"
        elif length <=16:
            return "uint16_t"
        elif length <=32:
            return "uint32_t"
        elif length <=64:
            return "uint64_t"
    
    def get_phys_signal_format(self, coding, length):
        
        #unsigned_sig_type = ["bool", "uint8", "uint16", "uint32", "uint64"]
        #signed_sig_type = ["int8", "int16", "int32", "int64", "float32", "float64"]

        is_positive_sig = False
        is_integer = False
        is_simple_sig = False

        try:
            factor = coding[1]
            offset = coding[0]
            if factor >= 0 and offset >= 0:
                is_positive_sig = True #simplest form of signal
            else:
                is_positive_sig = False #
            if factor.is_integer() and offset.is_integer():
                is_integer = True
            else:
                is_integer = False
                
        except:
            is_simple_sig = True #No conversion rule found, only positive values for lookup table

        if is_positive_sig and is_integer or is_simple_sig:
            # Is signal a positive integer signal or a signal with positive integer values for a lookup table
            if length == 1:
                return "bool"
            elif length <=8:
                return "uint8"
            elif length <=16:
                return "uint16"
            elif length <=32:
                return "uint32"
            elif length <=64:
                return "uint64"
        elif not is_positive_sig and is_integer:
            # Is signal a negative integer signal
            if length <=8-1:
                return "int8"
            elif length <=16-1:
                return "int16"
            elif length <=32-1:
                return "int32"
            elif length <=64-1:
                return "int64"
        else:
            # Is signal a positive/negative float signal
            if length <=32-1:
                return "float32"
            elif length <=64-1:
                return "float64"
            

    def get_frametriggering(self, conf_factory, frame_name):
        for _, value in conf_factory.__frame_triggerings__.items():
            if value.__frame__.__short_name__ == frame_name:
                return value

    def clear_content(self):
        self.__content__ = str()
        
    
    def append(self, text):
        self.__content__ += text + '\n'

    def write_msgspack(self, conf_factory):
        self.write_msgspack_cmakelists(conf_factory)
        self.write_msgpack_msgfiles(conf_factory)
        self.copy_file("templates/FROS/flexray_msgs/package.xml", self.target_dir + "ROS1/flexray_msgs/package.xml")
        self.copy_file("templates/FROS/flexray_msgs_ROS2/package.xml", self.target_dir + "ROS2/flexray_msgs/package.xml")
        
    def write_msgspack_cmakelists(self, conf_factory):    
               
        pdus = conf_factory.__pdus__.keys()
        msg_list = [s + '.msg' for s in pdus]
        
        #Write to ROS1
        environment = Environment(loader=FileSystemLoader("templates/FROS/flexray_msgs"))
        template = environment.get_template("CMakeLists.txt")
        content = template.render(
            msg_list=msg_list
        )
        filename = 'ROS1\\flexray_msgs\\CMakeLists.txt'
        self.write_content_to_file(filename, content)
        
        #Write to ROS2
        environment = Environment(loader=FileSystemLoader("templates/FROS/flexray_msgs_ROS2"))
        template = environment.get_template("CMakeLists.txt")
        content = template.render(
            msg_list = [re.sub(r"(_|-)+", " ", pdu_name).title().replace(" ", "").capitalize() for pdu_name in pdus]
        )
        filename = 'ROS2\\flexray_msgs\\CMakeLists.txt'
        self.write_content_to_file(filename, content)       
        
        
    
    def write_msgpack_msgfiles(self, conf_factory):
        environment_ros1 = Environment(loader=FileSystemLoader("templates/FROS/flexray_msgs"))
        template_ros1 = environment_ros1.get_template("PDU.msg")
        environment_ros2 = Environment(loader=FileSystemLoader("templates/FROS/flexray_msgs_ROS2"))
        template_ros2 = environment_ros2.get_template("PDU.msg")

        for pdu_name, pdu in conf_factory.__pdus__.items():
            signal_list = dict()
            for signal_instance_name, signal_instance in pdu.__signal_instances__.items():
                signal_name = signal_instance.__signal__.__name__ # Proper signal name
                type = self.get_phys_signal_format(signal_instance.__signal__.__compu_scale__, signal_instance.__signal__.__bit_length__)
                signal_list[signal_name] = type
   


            # Write to ROS1
            content_ros1 = template_ros1.render(signal_list=signal_list)
            filename= f"ROS1\\flexray_msgs\\msg\\{pdu_name}.msg"
            self.write_content_to_file(filename, content_ros1)
            
            # Write to ROS2
            content_ros2 = template_ros2.render(signal_list=signal_list)
            ros2_name = re.sub(r"(_|-)+", " ", pdu_name).title().replace(" ", "").capitalize()
            filename= f"ROS2\\flexray_msgs\\msg\\{ros2_name}.msg"
            self.write_content_to_file(filename, content_ros2)

    def write_interface(self, conf_factory):
        #Write ROS1
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("package.xml")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS1\\" + self.packagefolder + f"\\package.xml"
        self.write_content_to_file(filename, content)

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("CMakeLists.txt")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS1\\" + self.packagefolder + f"\\CMakeLists.txt"
        self.write_content_to_file(filename, content)

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw/include"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("ixxat_pkt.h")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS1\\" + self.packagefolder + f"\\include\\ixxat_pkt.h"
        self.write_content_to_file(filename, content)

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw/src"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("ixxat_gw.cpp")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS1\\" + self.packagefolder + f"\\src\\ixxat_gw.cpp"
        self.write_content_to_file(filename, content)

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw/launch"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("ixxat_gw.launch")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS1\\" + self.packagefolder + f"\\launch\\ixxat_gw.launch"
        self.write_content_to_file(filename, content)
        
        #Write ROS2            
        
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("package.xml")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS2\\" + self.packagefolder + f"\\package.xml"
        self.write_content_to_file(filename, content)

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("CMakeLists.txt")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS2\\" + self.packagefolder + f"\\CMakeLists.txt"
        self.write_content_to_file(filename, content)

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2/include"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("ixxat_pkt.h")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS2\\" + self.packagefolder + f"\\include\\ixxat_pkt.h"
        self.write_content_to_file(filename, content)

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2/src"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("ixxat_gw.cpp")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS2\\" + self.packagefolder + f"\\src\\ixxat_gw.cpp"
        self.write_content_to_file(filename, content)

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2/launch"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("ixxat_gw_launch.xml")        
        content = template.render(channel_name = self.channel_name.lower())
        filename = "ROS2\\" + self.packagefolder + f"\\launch\\ixxat_gw_launch.xml"
        self.write_content_to_file(filename, content)        
        
        self.write_defs(conf_factory)
        self.write_datahandler(conf_factory)

    def create_reserved_signal(self, target_bit_position, previous_signal_end_position):
        signal = dict()
        signal["length"] = target_bit_position - previous_signal_end_position
        signal["name"] = f'reserved_{previous_signal_end_position}_{signal["length"]}'
        signal["type"] = self.get_raw_signal_format(signal["length"])
        signal["bit-position"] = previous_signal_end_position
        return signal
    
    def fill_signal_list(self, signal_instances):
        import copy
        signals_output = copy.deepcopy(signal_instances)
        
        # Keeps track of where the last seen signal ends
        previous_signal_end_position = 0
        
        for signal in signal_instances:
            # Check if the current signal starts where the previous signal ended
            if signal["bit-position"] == previous_signal_end_position:
                # current signal starts where the previous signal ended
                # shift current bit position by the signal's lenght
                previous_signal_end_position += signal["length"]
            else:
                # We need to fill up with reserved bits
                # Calculate how many 32-bit reserved bits we need to add
                (multiples, mod) = divmod(signal["bit-position"]-previous_signal_end_position, 32)
                
                if multiples != 0 and mod == 0 : #only multiples of 32 to fill up
                    for j in range(0,multiples):
                        res_signal = self.create_reserved_signal( (j+1)*32+previous_signal_end_position, previous_signal_end_position)
                        signals_output.append(res_signal)
                        previous_signal_end_position += res_signal["length"]

                if multiples == 0 and mod!=0: #only a small mod is to fill up to 32 bit
                    res_signal = self.create_reserved_signal(previous_signal_end_position+mod, previous_signal_end_position)
                    signals_output.append(res_signal)
                    previous_signal_end_position += res_signal["length"] 

                if multiples !=0 and mod != 0: #multiples of 32 and a small mod is to fill up
                    for j in range(0,multiples):
                        res_signal = self.create_reserved_signal((j+1)*32+previous_signal_end_position, previous_signal_end_position)
                        signals_output.append(res_signal)
                        previous_signal_end_position += res_signal["length"] 
                    res_signal = self.create_reserved_signal(previous_signal_end_position+mod, previous_signal_end_position)
                    signals_output.append(res_signal)
                    previous_signal_end_position += res_signal["length"]

                # shift current bit position by the signal's lenght
                previous_signal_end_position += signal["length"]

        return sorted(signals_output, key=lambda d: d['bit-position'])
    
    def write_defs(self, conf_factory):
        pdudefs = dict()
        for pdu_name, pdu in conf_factory.__pdus__.items():
            pdudef = dict()
            signal_list = []
            pdudef["name"] = pdu_name
            for _, signal_instance in pdu.__signal_instances__.items():
                signal = dict()
                #coding = conf_factory.get_coding(signal_instance.__signal__.__coding_ref__)
                signal["name"] = signal_instance.__signal__.__name__ # Proper signal name
                signal["length"] = int(signal_instance.__signal__.__bit_length__)
                signal["type"] = self.get_raw_signal_format(signal["length"])
                signal["bit-position"] = int(signal_instance.__bit_position__)
                signal_list.append(signal)
            
            
            filled_signal_list = self.fill_signal_list(signal_list)

            pdudef["signal_list"] = filled_signal_list
            
            
            pdudefs[pdudef["name"]] = pdudef
        

        #Write ROS1
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw/include/"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("msgdefs.h")        
        content = template.render(pdudefs = pdudefs)
        filename = "ROS1\\" + self.packagefolder + f"\\include\\msgdefs.h"
        self.write_content_to_file(filename, content)
        
        #Write ROS2
        ros2_pdudefs = {}
        pdudefs_temp = copy.copy(pdudefs)
        
        for pdu_name, pdu in pdudefs_temp.items():
            pdu["name"] = re.sub(r"(_|-)+", " ", pdu_name).title().replace(" ", "").lower()
            ros2_pdudefs[pdu["name"]] = pdu
        
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2/include/"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("msgdefs.h")       
        content = template.render(pdudefs = ros2_pdudefs)
        filename = "ROS2\\" + self.packagefolder + f"\\include\\msgdefs.h"
        self.write_content_to_file(filename, content)

        # Include first step to decode multiplexer
        for muxpdu_name, muxpdu in conf_factory.__multiplexpdus__.items():
            muxpdudef = dict()
            signal_list = []
            muxpdudef["name"] = muxpdu.__short_name__ + "_demux"
            signal = dict()
            signal["name"] = muxpdu.__switch__.__short_name__
            signal["length"] = muxpdu.__switch__.__bit_length__
            signal["type"] = self.get_raw_signal_format(signal["length"])
            signal["bit-position"] = muxpdu.__switch__.__bit_position__
            signal_list.append(signal)
            filled_signal_list = self.fill_signal_list(signal_list)
            muxpdudef["signal_list"] = filled_signal_list
            pdudefs[muxpdudef["name"]] = muxpdudef

            # #Static Part of MuxPDU
            try:
                #Try if Multiplexer PDU has a static field defined
                staticmuxpdudef = dict()
                signal_list = []
                staticmuxpdudef["name"] = muxpdu.__static_pdu__.__short_name__
                staticmuxpdudef["offset"] = muxpdu.__static_segments__[0].__bit_position__
                for _, signal_instance in muxpdu.__static_pdu__.__signal_instances__.items():
                    signal = dict()
                    signal["name"] = signal_instance.__signal__.__name__ # Proper signal name
                    signal["length"] = int(signal_instance.__signal__.__bit_length__)
                    signal["type"] = self.get_raw_signal_format(signal["length"])
                    signal["bit-position"] = int(signal_instance.__bit_position__) + int(staticmuxpdudef["offset"])
                    signal_list.append(signal)
                filled_signal_list = self.fill_signal_list(signal_list)
                staticmuxpdudef["signal_list"] = filled_signal_list
                pdudefs[staticmuxpdudef["name"]] = staticmuxpdudef
            except:
                pass

            # #Switched Part of Multiplexer PDU
            for switchcode, muxpduinstance in muxpdu.__pdu_instances__.items():
                switchedpdu = dict()
                signal_list = []
                switchedpdu["name"] = muxpduinstance.__short_name__
                switchedpdu["offset"] = muxpdu.__segment_positions__[0].__bit_position__
                for _, signal_instance in muxpduinstance.__signal_instances__.items():
                    signal = dict()
                    signal["name"] = signal_instance.__signal__.__name__ # Proper signal name
                    signal["length"] = int(signal_instance.__signal__.__bit_length__)
                    signal["type"] = self.get_raw_signal_format(signal["length"])
                    signal["bit-position"] = int(signal_instance.__bit_position__) + int(switchedpdu["offset"])
                    signal_list.append(signal)
                filled_signal_list = self.fill_signal_list(signal_list)
                switchedpdu["signal_list"] = filled_signal_list
                pdudefs[switchedpdu["name"]] = switchedpdu

        #Write ROS1
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw/include/"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("pdudefs.h")        
        content = template.render(pdudefs = pdudefs, channel_name = self.channel_name.lower())
        filename = "ROS1\\" + self.packagefolder + f"\\include\\pdudefs.h"
        self.write_content_to_file(filename, content)
        
        #Write ROS2
        ros2_pdudefs = {}
        pdudefs_temp = copy.copy(pdudefs)
        for pdu_name, pdu in pdudefs_temp.items():
            pdu["name"] = re.sub(r"(_|-)+", " ", pdu_name).title().replace(" ", "").capitalize()
            ros2_pdudefs[pdu["name"]] = pdu

        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2/include/"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("pdudefs.h")         
        content = template.render(pdudefs = ros2_pdudefs, channel_name = self.channel_name.lower())
        filename = "ROS2\\" + self.packagefolder + f"\\include\\pdudefs.h"
        self.write_content_to_file(filename, content)

    def write_datahandler(self, conf_factory):
      
        import numpy as np
        # Array: Cycle, Slot
        fnktpntr_array = np.full((64, 309), "&DataHandler::blank", dtype=object)
        
        # Frame definition for definition of callable functions
        framedefs = []
        pdudefs = dict()
        muxpdudefs = dict()
        list_of_pdus = []
        list_of_publishers = []

        for _, frametriggering in conf_factory.__channels__[self.channel_name]["frametriggerings"].items():
        #for _, frame in conf_factory.__frames__.items():
            framedesc = dict()
            framedesc["pdus"] = []
            framedesc["name"] = frametriggering.__frame__.__short_name__
            for _, pdu_instance in frametriggering.__frame__.__pdu_instances__.items():
                pdu = dict()
                try:
                    # If a PDU does not contain any signals, it is skipped in fibex_parser, because for pdu in root.findall('.//fx:PDUS/fx:PDU/fx:SIGNAL-INSTANCES/..', self.__ns__): returns no results, so we need to check if this pdu instance has a pdu attached
                    pdu["name"] = pdu_instance.__pdu__.__short_name__
                except:
                    #print(f"Error in Frame {frame.__short_name__}")
                    continue
                pdu["offset"] = int(np.floor(pdu_instance.__bit_position__ / 8))
                pdu["update-bit"] = pdu_instance.__pdu_update_bit_position__
                (multiples, mod) = divmod(int(pdu_instance.__bit_position__), 8)
                if mod:
                    print(f'PDU not as 8 byte offset: {framedesc["name"]}')
                
                framedesc["pdus"].append(pdu)
            framedefs.append(framedesc)

            #triggering = self.get_frametriggering(conf_factory, framedesc["name"])

            #Functionpointer array
            for i in range(frametriggering.__base_cycle__, 63+1, frametriggering.__cycle_repetition__):
                fnktpntr_array[i][frametriggering.__slot_id__] = "&DataHandler::"+frametriggering.__frame__.__short_name__

            # PDU definitions for the unique PDU decoding functions
            for _, pduinstance in frametriggering.__frame__.__pdu_instances__.items():
                pdudef = dict()
                #Check if there is a PDU attached to this instance, it is not when e.g. there are no signals in this PDU
                try:
                    pdudef["name"] = pduinstance.__pdu__.__short_name__
                    #PDU is a valid PDU and thus should be included in the list of possible PDUs, so it gets its own publisher
                    #Avioid complete Multiplexer PDUs, as they are not defined as a whole PDU anywhere
                    list_of_pdus.append(pdudef["name"])
                    if not hasattr(pduinstance.__pdu__, '__static_pdu__'):
                         list_of_publishers.append(pdudef["name"])
                   
                except:
                    #PDU Instance without defined signals, simply ignore it
                    continue
                #Now differentiate between PDU and MultiplexPDU
                try:
                    #PDU
                    pdudef["signalinstances"] = pduinstance.__pdu__.__signal_instances__
                    #pdudefs.append(pdudef)
                    pdudefs.setdefault(pdudef["name"], pdudef)
                except:
                    #Multiplex PDU
                    muxpdudef = dict()
                    muxpdudef["name"] = pduinstance.__pdu__.__short_name__
                    muxpdudef["switchname"] = pduinstance.__pdu__.__switch__.__short_name__
                    muxpdudef["staticpdu"] = dict()
                    try:
                        #Try if Multiplexer PDU has a static field defined
                        muxpdudef["staticpdu"] = dict()
                        muxpdudef["staticpdu"]["name"] = pduinstance.__pdu__.__static_pdu__.__short_name__
                        muxpdudef["staticpdu"]["signalinstances"] = pduinstance.__pdu__.__static_pdu__.__signal_instances__
                        list_of_publishers.append(muxpdudef["staticpdu"]["name"])
                    except:
                        pass
                    switchedpdus = dict()
                    for switchcode, muxpduinstance in pduinstance.__pdu__.__pdu_instances__.items():
                        switchedpdu = dict()
                        switchedpdu["switchcode"] = switchcode
                        switchedpdu["name"] = muxpduinstance.__short_name__
                        switchedpdu["offset"] = pduinstance.__pdu__.__segment_positions__[0].__bit_position__ # calculate bit position as bytes
                        switchedpdu["signalinstances"] = muxpduinstance.__signal_instances__
                        switchedpdus[switchcode] = switchedpdu
                        list_of_publishers.append(switchedpdu["name"])
                    muxpdudef["switchedpdus"] = switchedpdus
                    #muxpdudefs.append(muxpdudef)
                    muxpdudefs.setdefault(muxpdudef["name"], muxpdudef)   

        #Sort list of PDUs for later on better reading
        list_of_pdus.sort()
        unique_list_of_pdus = np.unique(np.array(list_of_pdus))

        list_of_publishers.sort()
        unique_list_of_publishers = np.unique(np.array(list_of_publishers))

        muxpdudefs = dict(sorted(muxpdudefs.items()))
        pdudefs = dict(sorted(pdudefs.items()))

        #Write ROS1
        # Datahandler.cpp
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw/src/"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("datahandler.cpp")
        content = template.render(framedefs = framedefs, list_of_publishers=unique_list_of_publishers, pdudefs=pdudefs, muxpdudefs=muxpdudefs,  fnktpntr_array=fnktpntr_array.tolist(), channel_name = self.channel_name.lower())
        filename = "ROS1\\" + self.packagefolder + f"\\src\\datahandler.cpp"
        self.write_content_to_file(filename, content)
    
        # Datahandler.h
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw/src/"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("datahandler.h")
        content = template.render(framedefs = framedefs, list_of_pdus=unique_list_of_pdus, pdudefs=pdudefs, list_of_publishers=unique_list_of_publishers, fnktpntr_array=fnktpntr_array.tolist(), channel_name = self.channel_name.lower())
        filename = "ROS1\\" + self.packagefolder + f"\\src\\datahandler.h"
        self.write_content_to_file(filename, content)
        
        #Write ROS2
        # Datahandler.cpp
        ros2_list_of_pdus = np.array([])
        for pdu in unique_list_of_pdus:
            ros2_pdu = re.sub(r"(_|-)+", " ", pdu).title().replace(" ", "").capitalize()
            ros2_list_of_pdus = np.append(ros2_list_of_pdus, ros2_pdu)
        ros2_unique_list_of_pdus = np.unique(np.array(ros2_list_of_pdus))
            
        ros2_list_of_publishers = np.array([])
        for publisher in unique_list_of_publishers:
            ros2_publisher = re.sub(r"(_|-)+", " ", publisher).title().replace(" ", "").capitalize()
            ros2_list_of_publishers = np.append(ros2_list_of_publishers, ros2_publisher)
        ros2_unique_list_of_publishers = np.unique(np.array(ros2_list_of_publishers))
        
        ros2_pdudefs = {}
        for pdu_name, pdu in pdudefs.items():
            ros2_pdu = pdu
            ros2_pdu["name"] = re.sub(r"(_|-)+", " ", pdu_name).title().replace(" ", "").capitalize()
            ros2_pdudefs[ros2_pdu["name"]] = ros2_pdu
            
        ros2_muxpdudefs = {}
        for muxpdu_name, muxpdu in muxpdudefs.items():
            ros2_muxpdu = muxpdu
            ros2_muxpdu["name"] = re.sub(r"(_|-)+", " ", muxpdu_name).title().replace(" ", "").capitalize()
            if ros2_muxpdu["staticpdu"]: #some Multiplexer PDUs have no static PDU part
                ros2_muxpdu["staticpdu"]["name"] = re.sub(r"(_|-)+", " ", ros2_muxpdu["staticpdu"]["name"]).title().replace(" ", "").capitalize()
            
            for switchedpduname, pdu in ros2_muxpdu["switchedpdus"].items():
                pdu["name"] = re.sub(r"(_|-)+", " ", muxpdu_name).title().replace(" ", "").capitalize() + str(switchedpduname)
            ros2_pdudefs[ros2_muxpdu["name"]] = ros2_muxpdu
            ros2_muxpdudefs[ros2_muxpdu["name"]] = ros2_muxpdu
            
        ros2_framedefs = []
        for frame in framedefs:
            for pdu in frame["pdus"]:
                pdu["name"] = re.sub(r"(_|-)+", " ", pdu["name"]).title().replace(" ", "").capitalize()
            
            ros2_framedefs = np.append(ros2_framedefs, frame)
        
        
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2/src/"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("datahandler.cpp")
        content = template.render(framedefs = ros2_framedefs, list_of_publishers=ros2_unique_list_of_publishers, pdudefs=ros2_pdudefs, muxpdudefs=ros2_muxpdudefs,  fnktpntr_array=fnktpntr_array.tolist(), channel_name = self.channel_name.lower())
        filename = "ROS2\\" + self.packagefolder + f"\\src\\datahandler.cpp"
        self.write_content_to_file(filename, content)
    
        # Datahandler.h
        environment = Environment(loader=FileSystemLoader("templates/FROS/ixxat_gw_ROS2/src/"), trim_blocks=True, lstrip_blocks=True)
        template = environment.get_template("datahandler.h")
        content = template.render(framedefs = ros2_framedefs, list_of_pdus=ros2_unique_list_of_pdus, pdudefs=ros2_pdudefs, list_of_publishers=ros2_unique_list_of_publishers, fnktpntr_array=fnktpntr_array.tolist(), channel_name = self.channel_name.lower())
        filename = "ROS2\\" + self.packagefolder + f"\\src\\datahandler.h"
        self.write_content_to_file(filename, content)