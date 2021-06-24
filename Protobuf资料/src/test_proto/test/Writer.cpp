  #include <iostream>
  #include <fstream>
  #include "Mymessage.pb.h"

  int main()
  {
          Im::Content msg1;
           msg1.set_id(10);
           msg1.set_str("hello world");
           std::fstream output("./log", std::ios::out|std::ios::trunc|std::ios::binary);
          if(!msg1.SerializeToOstream(&output))
          {
                  std::cerr << "Failed to write msg." << std::endl;
                  return -1;
          }
          return 0;
 }
