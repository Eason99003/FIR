module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  reg                      awready,
    output  reg                      wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  reg                      arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  reg                      rvalid,
    output  reg  [(pDATA_WIDTH-1):0] rdata,

    // axi-stream slave for x[n] input
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                      ss_tready,

    // axi-stream slave for y[n] output 
    input   wire                     sm_tready, 
    output  reg                      sm_tvalid, 
    output  reg  [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                      sm_tlast, 
    
    // bram for tap RAM
    output  reg  [3:0]               tap_WE,
    output  reg                      tap_EN,
    output  reg  [(pDATA_WIDTH-1):0] tap_Di,
    output  reg  [(pADDR_WIDTH-1):0] tap_A, 
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  reg  [3:0]               data_WE,
    output  reg                      data_EN,
    output  reg  [(pDATA_WIDTH-1):0] data_Di,
    output  reg  [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

reg ap_start, ap_start_next;
reg ap_idle, ap_idle_next;
reg ap_done, ap_done_next;

reg [(pDATA_WIDTH-1):0] data_length;
reg [(pDATA_WIDTH-1):0] tap_number;

reg [2:0] fir_state, fir_state_next;

// axi-lite read
localparam  AXI_READ_IDLE = 2'd0,
            AXI_READ_ADDR = 2'd1,
            AXI_READ_WAIT = 2'd2,
            AXI_READ_DATA = 2'd3;

// axi-lite write
localparam  AXI_WRITE_IDLE = 2'd0,
            AXI_WRITE_ADDR = 2'd1,
            AXI_WRITE_DATA = 2'd2;

// stream-in
// reg ss_tready_next

reg last_flg;

// AXI
reg [1:0] axi_read_state, axi_read_state_next;
reg [1:0] axi_write_state, axi_write_state_next;

// tapRAM
reg [(pADDR_WIDTH-1):0] tap_access_addr;
reg [(pDATA_WIDTH-1):0] real_Do;
reg [(pADDR_WIDTH-1):0] tapWriteAddr;
reg [(pADDR_WIDTH-1):0] tapReadAddr;

// dataRAM
reg [(pADDR_WIDTH-1):0] dataRam_rst_cnt, dataRam_rst_cnt_next;
reg [(pADDR_WIDTH-1):0] dataWriteAddr, dataWriteAdddr_next;
reg [(pADDR_WIDTH-1):0] dataReadAddr, dataReadAddr_next;

// store the temporary computaion result
reg signed [(pDATA_WIDTH-1):0] h, h_next, a, a_next;
reg signed [(pDATA_WIDTH-1):0] y, y_next, m, m_next;

reg [(pADDR_WIDTH-1):0] k, k_next;
reg [2:0] cal_count, cal_count_next;

localparam FIR_IDLE = 3'b000,
           DATA_RST = 3'b001,
           FIR_WAIT = 3'b010,
           FIR_SSIN = 3'b011,
           FIR_STORE = 3'b100,
           FIR_RUN = 3'b101,
           FIR_CAL = 3'b110,
           FIR_OUT = 3'b111;

// AXI-Lite Read
always @* begin
  case (axi_read_state)
    AXI_READ_IDLE: axi_read_state_next = (arvalid == 1) ? AXI_READ_ADDR : AXI_READ_IDLE;
    AXI_READ_ADDR: axi_read_state_next = AXI_READ_WAIT;
    AXI_READ_WAIT: axi_read_state_next = AXI_READ_DATA;
    AXI_READ_DATA: axi_read_state_next = (rvalid == 1 && rready == 1) ? AXI_READ_IDLE : AXI_READ_DATA;
    default: axi_read_state_next = AXI_READ_IDLE;
  endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    axi_read_state <= AXI_READ_IDLE;
    arready <= 0;
    rvalid <= 0;
  end else begin
    axi_read_state <= axi_read_state_next;
    arready <= (axi_write_state_next == AXI_READ_ADDR) ? 1 : 0;
    rvalid <= (axi_write_state_next == AXI_READ_DATA) ? 1 : 0;
  end
end

always @* begin
    case (tapReadAddr)
      'h00: rdata = {ap_idle, ap_done, ap_start};
      'h10: rdata = data_length;
      'h14: rdata = tap_number;
      default: rdata = tap_Do;
    endcase
end 

// AXI-Lite Write
always @* begin
  case (axi_write_state) 
    AXI_WRITE_IDLE: axi_write_state_next = (awvalid == 1) ? AXI_WRITE_ADDR : AXI_WRITE_IDLE;
    AXI_WRITE_ADDR: axi_write_state_next = AXI_WRITE_DATA;
    AXI_WRITE_DATA: axi_write_state_next = (wvalid == 1) ? AXI_WRITE_IDLE : AXI_WRITE_DATA;
    default: axi_write_state_next = AXI_WRITE_IDLE;
  endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    axi_write_state <= AXI_WRITE_IDLE;
    awready <= 0;
    wready <= 0;
  end else begin
    axi_write_state <= axi_write_state_next;
    awready <= (axi_write_state_next == AXI_WRITE_ADDR) ? 1 : 0;
    wready <= (axi_write_state_next == AXI_WRITE_DATA) ? 1 : 0;
  end
end

// tap RAM address control
always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    tapWriteAddr <= 0;
    tapReadAddr <= 0;
  end else begin
    tapWriteAddr <= (awvalid == 1 && awready == 1) ? awaddr : tapWriteAddr;
    tapReadAddr <= (arvalid == 1 && arready == 1) ? araddr : tapReadAddr;
  end
end

// Stream-in
always @* begin
  if (fir_state == FIR_SSIN) ss_tready = 1;
  else ss_tready = 0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    last_flg <= 0;
  end else begin
    if (fir_state == FIR_IDLE) last_flg <= 0;
    else if (fir_state == FIR_SSIN) last_flg <= ss_tlast;
    else last_flg <= last_flg;
  end
end

// Stream-out
always @* begin
  if (fir_state == FIR_OUT) begin
    sm_tvalid = 1;
    sm_tdata = y;
    sm_tlast = last_flg;
  end else begin
    sm_tvalid = 0;
    sm_tdata = 0;
    sm_tlast = 0;
  end
end

// block level
always @* begin
  if (tapWriteAddr == 'h00 && wready == 1 && wvalid == 1 && wdata == 1) begin
    ap_start_next = wdata[0];
  end else if (fir_state == FIR_SSIN) begin
    ap_start_next = 0;
  end else begin
    ap_start_next = ap_start;
  end
end

always @* begin
  if (tapWriteAddr == 'h00 && wready == 1 && wvalid == 1 && wdata == 1) begin
    ap_idle_next = 0;
  end else if (fir_state == FIR_OUT && last_flg == 1) begin
    ap_idle_next = 1;
  end else begin
    ap_idle_next = ap_idle;
  end
end

always @* begin
  if (fir_state == FIR_OUT && last_flg == 1) begin
    ap_done_next = 1;
  end else if (fir_state == FIR_SSIN) begin
    ap_done_next = 0;
  end else if (tapReadAddr == 'h00) begin
    ap_done_next = 0;
  end else begin
    ap_done_next = ap_done;
  end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    ap_start <= 0;
    ap_idle <= 1;
    ap_done <= 0;
    data_length <= 0;
    tap_number <= 0;
  end else begin
    ap_start <= ap_start_next;
    ap_idle <= ap_idle_next;
    ap_done <= ap_done_next;
    data_length <= (tapWriteAddr == 'h10 && wready == 1 && wvalid == 1) ? wdata : data_length;
    tap_number <= (tapWriteAddr == 'h14 && wready == 1 && wvalid == 1) ? wdata : tap_number;
  end
end

// tap-Ram
always @* begin
  if (wready == 1 && wvalid == 1) tap_access_addr = tapWriteAddr;
  else if (axi_read_state == AXI_READ_WAIT && ap_idle == 1) tap_access_addr = tapReadAddr;
  else tap_access_addr = 4 * k + 'h80;
  tap_A = (tap_access_addr >= 'h80 && tap_access_addr <= 'hFC) ? tap_access_addr - 'h80 : 0;
end

always @* begin
  tap_EN = (
    (wready == 1 && wvalid == 1) ||
    (rready == 1 && rvalid == 1) ||
    (fir_state == FIR_RUN)
  ) ? 1 : 0;
end

always @* begin
  if (wready == 1 && wvalid == 1 && tapWriteAddr != 'h00 && tapWriteAddr != 'h10) begin
    tap_WE = 4'b1111;
    tap_Di = wdata;
  end else begin
    tap_WE = 0;
    tap_Di = 0;
  end
end

// data RAM address
always @* begin
  if (fir_state == FIR_IDLE) begin
    dataWriteAdddr_next = 0;
  end else if (fir_state == FIR_SSIN) begin
    if (dataWriteAddr == tap_number - 1) dataWriteAdddr_next = 0;
    else dataWriteAdddr_next = dataWriteAddr + 1;
  end else begin
    dataWriteAdddr_next = dataWriteAddr;
  end
end

always @* begin
  if (fir_state == FIR_IDLE) begin
    dataReadAddr_next = 0;
  end else if (fir_state == FIR_SSIN) begin
    dataReadAddr_next = dataWriteAddr;
  end else if (fir_state_next == FIR_RUN) begin
    dataReadAddr_next = 
      (dataWriteAddr - k >= 0) ? (dataWriteAddr - k) : (tap_number + dataWriteAddr - k);
  end else begin
    dataReadAddr_next = dataReadAddr;
  end
end

always @* begin
  if (fir_state == FIR_RUN) begin
    if (k == (tap_number - 1)) k_next = 0;
    else k_next = k + 1;
  end else begin
    k_next = 0;
  end
end

always @* begin
  if (fir_state == DATA_RST) dataRam_rst_cnt_next = dataRam_rst_cnt + 1;
  else dataRam_rst_cnt_next = 0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    dataRam_rst_cnt <= 0;
    dataWriteAddr <= 0;
    dataReadAddr <= 0;
    k <= 0;
  end else begin
    dataRam_rst_cnt <= dataRam_rst_cnt_next;
    dataWriteAddr <= dataWriteAdddr_next;
    dataReadAddr <= dataReadAddr_next;
    k <= k_next;
  end
end

// data RAM control
always @* begin
  if (fir_state == DATA_RST) begin
    data_EN = 1;
    data_A = (dataRam_rst_cnt << 2);
    data_WE = 4'b1111;
    data_Di = 0;
  end else if (fir_state == FIR_SSIN) begin
    data_EN = 1;
    data_A = (dataWriteAddr << 2);
    data_WE = 4'b1111;
    data_Di = ss_tdata;
  end else if (fir_state == FIR_RUN) begin
    data_EN = 1;
    data_A = (dataReadAddr << 2);
    data_WE = 4'b0000;
    data_Di = 0;
  end else begin
    data_EN = 0;
    data_A = 0;
    data_WE = 0;
    data_Di = 0;
  end
end

always @* begin
  if (fir_state == FIR_CAL) begin
    if (cal_count == 2'd2) cal_count_next = 0;
    else cal_count_next = cal_count + 1;
  end else begin
    cal_count_next = 0;
  end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) cal_count <= 0;
  else cal_count <= cal_count_next;
end

// FIR fsm
always @* begin
  case (fir_state)
    FIR_IDLE: fir_state_next = (wready == 1 && wvalid == 1 && wdata == 1) ? DATA_RST : FIR_IDLE;
    DATA_RST: fir_state_next = (dataRam_rst_cnt == (tap_number - 1)) ? FIR_WAIT : DATA_RST;
    FIR_WAIT: fir_state_next = (ss_tvalid == 1) ? FIR_SSIN : FIR_WAIT;
    FIR_SSIN: fir_state_next = (ss_tready == 1) ? FIR_STORE : FIR_SSIN;
    FIR_STORE: fir_state_next = FIR_RUN;
    FIR_RUN: fir_state_next = (k == (tap_number - 1)) ? FIR_CAL : FIR_RUN;
    FIR_CAL: fir_state_next = (cal_count == 2'd2) ? FIR_OUT : FIR_CAL;
    FIR_OUT: begin
      if (last_flg == 1) fir_state_next = FIR_IDLE;
      else if (sm_tready) fir_state_next = FIR_WAIT;
      else fir_state_next = FIR_OUT;
    end
    default: fir_state_next = fir_state;
  endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) fir_state <= FIR_IDLE;
  else fir_state <= fir_state_next;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    y <= 0;
    m <= 0;
    h <= 0;
    a <= 0;
  end else if (fir_state == FIR_SSIN) begin
    y <= 0;
    m <= 0;
    h <= 0;
    a <= 0;
  end else if (fir_state == FIR_RUN || fir_state == FIR_CAL) begin
    y <= y_next;
    m <= m_next;
    h <= h_next;
    a <= a_next;
  end else begin
    y <= y;
    m <= m;
    h <= h;
    a <= a;
  end
end


always @* begin
  y_next = y + m;
  m_next = h * a;
  h_next = tap_Do;
  a_next = data_Do;
end

endmodule
