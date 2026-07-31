#include "hal_data.h"

void hal_entry(void)
{
    void knl_start_mtkernel(void);

    knl_start_mtkernel();
}
