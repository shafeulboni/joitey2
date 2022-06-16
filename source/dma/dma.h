
#ifndef DMA_H
#define DMA_H

void InitDma();
void DMA_config(uint32_t mem_address);
void StartTransaction();
void ConfigureOePin();

#endif //DMA_H
