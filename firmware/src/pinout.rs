macro_rules! pinout {
    ($p:ident . wf_sck) => ($p.P1_00);
    ($p:ident . wf_miso) => ($p.P0_22);
    ($p:ident . wf_mosi) => ($p.P1_07);
    ($p:ident . wf_cs) => ($p.P0_06);
    ($p:ident . wf_fod) => ($p.P1_13);

    ($p:ident . nf_sck) => ($p.P0_05);
    ($p:ident . nf_miso) => ($p.P0_04);
    ($p:ident . nf_mosi) => ($p.P0_26);
    ($p:ident . nf_cs) => ($p.P0_08);
    ($p:ident . nf_fod) => ($p.P0_12);
}
