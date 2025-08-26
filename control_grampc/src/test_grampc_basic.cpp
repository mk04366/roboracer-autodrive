#include "control_grampc/mpcc_model.h"
extern "C" {
#include "grampc.h"
}

#include <iostream>

int main() {
    std::cout << "Testing basic GRAMPC initialization..." << std::endl;
    
    // Test context
    mpcc_ctx_t ctx;
    ctx.L = 0.33;
    ctx.v_max = 1.0;
    
    // Try to initialize GRAMPC
    TYPE_GRAMPC_POINTER(grampc);
    grampc = nullptr;
    
    std::cout << "About to call grampc_init..." << std::endl;
    grampc_init(&grampc, (void*)&ctx);
    std::cout << "grampc_init completed" << std::endl;
    
    if (!grampc) {
        std::cout << "GRAMPC initialization failed!" << std::endl;
        return 1;
    }
    
    std::cout << "GRAMPC initialized successfully" << std::endl;
    
    // Test basic parameter setting
    grampc->param->Nx = 4;
    grampc->param->Nu = 2; 
    grampc->param->Ng = 1;
    grampc->param->Nh = 0;
    grampc->param->Np = 0;
    
    std::cout << "Parameters set successfully" << std::endl;
    
    // Clean up
    grampc_free(&grampc);
    std::cout << "Test completed successfully!" << std::endl;
    
    return 0;
}
