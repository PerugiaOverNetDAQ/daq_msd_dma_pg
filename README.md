# Git submodule workflow
1. Clone the repository with the `--recursive` option:
  ```
  git clone --recursive git@github.com:PerugiaOverNetDAQ/daq_msd_dma_pg.git
  ```
2. If you already have cloned the repository without the `--recursive` flag:
  ```
  git submodule init
  git submodule update
  ```
3. To update the repository:
  ```
  git pull
  git submodule update
  ```

# Note
There were corrupted files/folders that reported the error
```
Sorry, there was a problem downloading some files from OneDrive. Please try again.
https://onedrive.live.com/?cid=6ed690b84d0021af&id=6ED690B84D0021AF%21237016&action=Download&authKey=!AD03LlWghrVBFJE
```
Here is the list of corrupted files/folders:

- sim
- _lib1_0.qpg
- Fifo.qip
- Fifo_TX_Regs.qip
- FifoRX.qip
- Internal_Fifo.qip
- metadata_Fifo.qip
- RegEventFifo.qip
- SC_FIFO.qip
- DE10DAQ_description.txt
