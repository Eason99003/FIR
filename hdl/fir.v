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

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

// tapRAM
reg [3:0] tap_WE;
reg tap_EN;
reg [(pDATA_WIDTH-1):0] tap_Di;
reg [(pADDR_WIDTH-1):0] tap_A; 
wire [(pDATA_WIDTH-1):0] tap_Do;
reg [(pADDR_WIDTH-1):0] tap_access_addr;
reg [(pADDR_WIDTH-1):0] tapWriteAddr;
reg [(pADDR_WIDTH-1):0] tapReadAddr;

// dataRAM
reg [3:0] data_WE;
reg data_EN;
reg [(pDATA_WIDTH-1):0] data_Di;
reg [(pADDR_WIDTH-1):0] data_A;
wire [(pDATA_WIDTH-1):0] data_Do;
reg [(pDATA_WIDTH-1):0] dataRam_rst_cnt, dataRam_rst_cnt_next;
reg [(pADDR_WIDTH-1):0] dataWriteAddr, dataWriteAddr_next;
reg [(pADDR_WIDTH-1):0] dataReadAddr, dataReadAddr_next;

bram32 tap_RAM (
    .CLK(axis_clk),
    .WE(tap_WE),
    .EN(tap_EN),
    .Di(tap_Di),
    .A(tap_A),
    .Do(tap_Do)
);

bram32 data_RAM(
    .CLK(axis_clk),
    .WE(data_WE),
    .EN(data_EN),
    .Di(data_Di),
    .A(data_A),
    .Do(data_Do)
);

reg ap_start, ap_start_next;
reg ap_idle, ap_idle_next;
reg ap_done, ap_done_next;
reg [(pDATA_WIDTH-1):0] data_length_next, tap_number_next;
reg [(pDATA_WIDTH-1):0] data_count, data_count_next;

reg [(pDATA_WIDTH-1):0] data_length;
reg [(pDATA_WIDTH-1):0] tap_number;

reg [2:0] fir_state, fir_state_next;


reg last_flg;


// AXI
reg arready_next, read_req, read_req_next, rvalid_next;
reg awready_next, wready_next;
reg [1:0] axi_write_state, axi_write_state_next;


// store the temporary computaion result
reg signed [(pDATA_WIDTH-1):0] h, h_next, a, a_next;
reg signed [(pDATA_WIDTH-1):0] y, y_next, m, m_next;

reg [(pDATA_WIDTH-1):0] k, k_next;

reg [(pDATA_WIDTH-1):0] x_buffer;
reg x_buffer_count;

reg [(pDATA_WIDTH-1):0] y_buffer;
reg y_buffer_count;

localparam FIR_IDLE = 3'b000,
           DATA_RST = 3'b001,
           FIR_WAIT = 3'b010,
           FIR_SSIN = 3'b011,
           FIR_RUN = 3'b100,
           FIR_CAL = 3'b101,
           FIR_OUT = 3'b110;


// AXI-Lite Read
always @* begin
  arready_next = (arvalid == 1 && ~arready) ? 1 : 0;
  read_req_next = (arvalid == 1 && arready == 1) ? 1 : 0;
  rvalid_next = (read_req == 1 || ((rvalid == 1) && ~rready)) ? 1 : 0;
end


always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    arready <= 0;
    rvalid <= 0;
    read_req <= 0;
  end else begin
    arready <= arready_next;
    rvalid <= rvalid_next;
    read_req <= read_req_next;
  end
end

always @* begin
  if (rvalid == 1 && rready == 1) begin
    case (tapReadAddr)
      'h00: rdata = {ap_idle, ap_done, ap_start};
      'h10: rdata = data_length;
      'h14: rdata = tap_number;
      default: begin
        if (ap_idle == 1) rdata = tap_Do;
        else rdata = 32'hffffffff;
      end
    endcase
  end else begin
    rdata = 0;
  end
end

// AXI-Lite Write
always @* begin
  awready_next = (awvalid == 1 && ~awready) ? 1 : 0;
  wready_next = (awready == 1 || (wready && ~wvalid));
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    awready <= 0;
    wready <= 0;
  end else begin
    awready <= awready_next;
    wready <= wready_next;
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


always @* begin
  if (x_buffer_count == 0) ss_tready = 1;
  else ss_tready = 0;
end

// x buffer
always @(posedge axis_clk) begin
  if (~axis_rst_n) begin
    x_buffer <= 0;
    x_buffer_count <= 0;
  end else if (ss_tready && ss_tvalid) begin
    x_buffer <= ss_tdata;
    x_buffer_count <= 1;
  end else if (fir_state == FIR_SSIN) begin
    x_buffer <= 0;
    x_buffer_count <= 0;
  end else begin
    x_buffer = x_buffer;
    x_buffer_count <= x_buffer_count;
  end
end



always @* begin
  if (y_buffer_count == 1) begin
    sm_tvalid = 1;
    sm_tdata = y_buffer;
  end else begin
    sm_tvalid = 0;
    sm_tdata = 0;
 end
end

// y buffer
always @(posedge axis_clk) begin
  if (~axis_rst_n) begin
    y_buffer <= 0;
    y_buffer_count <= 0;
  end else if (fir_state == FIR_CAL && y_buffer_count == 0) begin
    y_buffer <= y_next;
    y_buffer_count <= 1;
  end else if (fir_state == FIR_OUT && y_buffer_count == 1) begin
    if (sm_tvalid && sm_tready) begin
      y_buffer <= y;
      y_buffer_count <= 1;
    end else begin
      y_buffer <= y_buffer;
      y_buffer_count <= y_buffer_count;
    end
  end else if (sm_tvalid && sm_tready) begin
    y_buffer <= 0;
    y_buffer_count <= 0;
  end else begin
    y_buffer <= y_buffer;
    y_buffer_count <= y_buffer_count;
  end
end



// block level
always @* begin
  if (tapWriteAddr == 'h00 && wready == 1 && wvalid == 1 
    && wdata == 1 && ap_idle == 1) begin
    ap_start_next = 1;
  end else begin
    ap_start_next = 0;
  end
end

always @* begin
  if (tapWriteAddr == 'h00 && wready == 1 && wvalid == 1 
    && wdata == 1 && ap_idle == 1) begin
    ap_idle_next = 0;
  end else if (fir_state_next == FIR_IDLE && data_count[pDATA_WIDTH-1:0] == data_length[pDATA_WIDTH-1:0]) begin
    ap_idle_next = 1;
  end else begin
    ap_idle_next = ap_idle;
  end
end

always @* begin
  if (fir_state_next == FIR_IDLE && data_count[pDATA_WIDTH-1:0] == data_length[pDATA_WIDTH-1:0]) begin
    ap_done_next = 1;
  end else if (fir_state == FIR_IDLE) begin
    if (tapReadAddr == 'h00 && rvalid == 1 && rready == 1) begin
      ap_done_next = 0;
    end else begin
      ap_done_next = ap_done;
    end
  end else if (fir_state == DATA_RST) begin
    ap_done_next = 0;
  end else begin
    ap_done_next = ap_done;
  end
end

always @* begin
  if (ap_idle == 1) begin
    data_length_next = 
      (tapWriteAddr == 'h10 && wready == 1 && wvalid == 1) ? wdata : data_length;
    tap_number_next =
      (tapWriteAddr == 'h14 && wready == 1 && wvalid == 1) ? wdata : tap_number;
  end else begin
    data_length_next = data_length;
    tap_number_next = tap_number;
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
    data_length <= data_length_next;
    tap_number <= tap_number_next;
  end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    data_count <= 0;
  end else if (fir_state == FIR_IDLE) begin
    data_count <= 0;
  end else begin
    data_count <= data_count_next;
  end
end

always @* begin
  if (fir_state_next == FIR_SSIN) begin
    data_count_next = data_count + 1;
  end else begin
    data_count_next = data_count;
  end
end

/////////////////
// tap-Ram
always @* begin
  if (ap_idle == 1) begin
    if (wready == 1 && wvalid == 1) tap_access_addr = tapWriteAddr;
    else if ((read_req == 1) || (rvalid == 1 && ~rready)) tap_access_addr = tapReadAddr;
    else tap_access_addr = 0;
    tap_A = (tap_access_addr >= 'h80 && tap_access_addr <= 'hFC) ? tap_access_addr[6:0] : 0;
  end else begin
    if (k == tap_number) tap_A = 0;
    else tap_A = k << 2;
  end
end

always @* begin
  if (ap_idle == 1) begin
    tap_EN = (
    (wready == 1 && wvalid == 1) ||
    ((read_req == 1 || (rvalid)))
    ) ? 1 : 0;
  end else if (fir_state_next == FIR_SSIN || fir_state == FIR_SSIN || fir_state == FIR_RUN) begin
    tap_EN = 1;
  end else begin
    tap_EN = 0;
  end
end 

always @* begin
  if (wready == 1 && wvalid == 1 && (tapWriteAddr >= 'h80 && tapWriteAddr <= 'hFC) && (ap_idle == 1)) begin
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
    dataWriteAddr_next = 0;
  end else if (k == tap_number - 1) begin
    if (dataWriteAddr == tap_number - 1) dataWriteAddr_next = 0;
    else dataWriteAddr_next = dataWriteAddr + 1;
  end else begin
    dataWriteAddr_next = dataWriteAddr;
  end
end

always @* begin
  if (fir_state == FIR_IDLE) begin
    dataReadAddr_next = 0;
  end else if (fir_state_next == FIR_SSIN || fir_state_next == FIR_RUN) begin
    if (dataWriteAddr == 0) begin
      dataReadAddr_next = tap_number[(pADDR_WIDTH-1):0] - k_next;
    end else begin
      dataReadAddr_next = (dataWriteAddr >= k_next) ? 
        (dataWriteAddr - k_next) : (tap_number[(pADDR_WIDTH-1):0] + (dataWriteAddr - k_next));
    end
  end else begin
    dataReadAddr_next = dataReadAddr;
  end
end

always @* begin
  if (fir_state_next == FIR_SSIN || fir_state_next == FIR_RUN) begin
    if (k == (tap_number)) begin 
      k_next = 0;
    end else begin
      k_next = k + 1;
    end
  end else begin
    k_next = 0;
  end
end

always @* begin
  if (fir_state_next == DATA_RST) begin
    if (dataRam_rst_cnt == tap_number) dataRam_rst_cnt_next = dataRam_rst_cnt;
    else dataRam_rst_cnt_next = dataRam_rst_cnt + 1;
  end else begin
    dataRam_rst_cnt_next = 0;
  end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (~axis_rst_n) begin
    dataRam_rst_cnt <= 0;
    dataWriteAddr <= 0;
    dataReadAddr <= 0;
    k <= 0;
  end else begin
    dataRam_rst_cnt <= dataRam_rst_cnt_next;
    dataWriteAddr <= dataWriteAddr_next;
    dataReadAddr <= dataReadAddr_next;
    k <= k_next;
  end
end

// data RAM control
always @* begin
  if (fir_state_next == DATA_RST) begin
    data_EN = 1;
    data_A = (dataRam_rst_cnt << 2);
    data_WE = 4'b1111;
    data_Di = 0;
  end else if (fir_state_next == FIR_SSIN) begin
    data_EN = 1;
    data_A = (dataWriteAddr << 2);
    data_WE = 4'b1111;
    data_Di = x_buffer;
  end else if (fir_state == FIR_SSIN || fir_state == FIR_RUN) begin
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


// FIR fsm
always @* begin
  case (fir_state)
    FIR_IDLE: fir_state_next = (ap_start == 1) ? DATA_RST : FIR_IDLE;
    DATA_RST: begin
      if (dataRam_rst_cnt == tap_number) begin
        fir_state_next = FIR_WAIT;
      end else begin
        fir_state_next = DATA_RST;
      end
    end
    FIR_WAIT: fir_state_next = (x_buffer_count == 1) ? FIR_SSIN : FIR_WAIT;
    FIR_SSIN: fir_state_next = FIR_RUN;
    FIR_RUN: fir_state_next = (k == tap_number) ? FIR_CAL : FIR_RUN;
    FIR_CAL: begin
      if (y_buffer_count == 0 && x_buffer_count == 1) begin
        if (data_count[pDATA_WIDTH-1:0] == data_length[pDATA_WIDTH-1:0]) fir_state_next = FIR_IDLE;
        else fir_state_next = FIR_SSIN;
      end else if (y_buffer_count == 0 && x_buffer_count == 0) begin
        if (data_count[pDATA_WIDTH-1:0] == data_length[pDATA_WIDTH-1:0]) fir_state_next = FIR_IDLE;
        else fir_state_next = FIR_WAIT;
      end 
    end
    FIR_OUT: begin
      if (sm_tready && sm_tvalid) begin
        if (data_count[pDATA_WIDTH-1:0] == data_length[pDATA_WIDTH-1:0]) fir_state_next = FIR_IDLE;
        else if (x_buffer_count == 1) fir_state_next = FIR_SSIN;
        else fir_state_next = FIR_WAIT;
      end 
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
    m <= m_next;
    h <= h_next;
    a <= a_next;
  end else begin
    y <= y_next;
    m <= m_next;
    h <= h_next;
    a <= a_next;
  end
end

always @* begin
    y_next = y + m;
    m_next = h * a;
    if (fir_state == FIR_SSIN) begin
      a_next = x_buffer;
      h_next = tap_Do;
    end else if (fir_state == FIR_RUN || fir_state == FIR_CAL) begin
      a_next = data_Do;
      h_next = tap_Do;
    end else begin
      a_next = 0;
      h_next = 0;
    end
end

endmodule
