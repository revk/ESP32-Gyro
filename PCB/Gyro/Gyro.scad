// Generated case design for Gyro/Gyro.kicad_pcb
// By https://github.com/revk/PCBCase
// Generated 2025-06-17 08:20:53
// title:	Glove
// rev:	5
// company:	Adrian Kennard Andrews & Arnold Ltd
// comment:	www.me.uk
// comment:	@TheRealRevK
//

// Globals
margin=0.200000;
lip=2.000000;
casebottom=2.000000;
casetop=7.000000;
casewall=3.000000;
fit=0.000000;
edge=2.000000;
pcbthickness=1.200000;
nohull=false;
hullcap=1.000000;
hulledge=1.000000;
useredge=false;
spacing=57.000000;
pcbwidth=41.000000;
pcblength=24.000000;
originx=100.500000;
originy=102.000000;

module outline(h=pcbthickness,r=0){linear_extrude(height=h)offset(r=r)polygon(points=[[11.500000,10.650000],[11.500000,12.000000],[19.500000,12.000000],[19.758819,11.965925],[20.000000,11.866025],[20.207107,11.707106],[20.366026,11.500000],[20.465927,11.258819],[20.500000,11.000000],[20.500000,-11.000000],[20.465926,-11.258819],[20.366026,-11.500000],[20.207107,-11.707107],[20.000000,-11.866026],[19.758819,-11.965926],[19.500000,-12.000000],[-19.500000,-12.000000],[-19.758819,-11.965926],[-20.000000,-11.866026],[-20.207107,-11.707107],[-20.366026,-11.500000],[-20.465926,-11.258819],[-20.500000,-11.000000],[-20.500000,11.000000],[-20.465926,11.258819],[-20.366026,11.500000],[-20.207107,11.707107],[-20.000000,11.866026],[-19.758819,11.965926],[-19.500000,12.000000],[-4.500000,12.000000],[-4.500000,10.650000],[-4.382843,10.367157],[-4.100000,10.250000],[11.100000,10.250000],[11.382843,10.367157]],paths=[[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35]]);}

module pcb(h=pcbthickness,r=0){linear_extrude(height=h)offset(r=r)polygon(points=[[11.500000,10.650000],[11.500000,12.000000],[19.500000,12.000000],[19.758819,11.965925],[20.000000,11.866025],[20.207107,11.707106],[20.366026,11.500000],[20.465927,11.258819],[20.500000,11.000000],[20.500000,-11.000000],[20.465926,-11.258819],[20.366026,-11.500000],[20.207107,-11.707107],[20.000000,-11.866026],[19.758819,-11.965926],[19.500000,-12.000000],[-19.500000,-12.000000],[-19.758819,-11.965926],[-20.000000,-11.866026],[-20.207107,-11.707107],[-20.366026,-11.500000],[-20.465926,-11.258819],[-20.500000,-11.000000],[-20.500000,11.000000],[-20.465926,11.258819],[-20.366026,11.500000],[-20.207107,11.707107],[-20.000000,11.866026],[-19.758819,11.965926],[-19.500000,12.000000],[-4.500000,12.000000],[-4.500000,10.650000],[-4.382843,10.367157],[-4.100000,10.250000],[11.100000,10.250000],[11.382843,10.367157]],paths=[[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35]]);}
module part_D1(part=true,hole=false,block=false)
{
translate([-5.000000,-3.500000,1.200000])rotate([0,0,90.000000])m0(part,hole,block,casetop); // D1 (back)
};
module part_J3(part=true,hole=false,block=false)
{
translate([-12.500000,-5.000000,1.200000])m1(part,hole,block,casetop); // J3 (back)
};
module part_R1(part=true,hole=false,block=false)
{
translate([-5.000000,7.000000,1.200000])rotate([0,0,90.000000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_C2(part=true,hole=false,block=false)
{
translate([15.562500,-0.050000,1.200000])rotate([0,0,180.000000])m3(part,hole,block,casetop); // C2 (back)
};
module part_D20(part=true,hole=false,block=false)
{
translate([12.000000,-4.500000,1.200000])m0(part,hole,block,casetop); // D1 (back)
};
module part_C26(part=true,hole=false,block=false)
{
translate([0.000000,-6.500000,1.200000])rotate([0,0,180.000000])m4(part,hole,block,casetop); // RevK:C_0805 C_0805_2012Metric (back)
};
module part_V1(part=true,hole=false,block=false)
{
};
module part_U3(part=true,hole=false,block=false)
{
translate([3.450000,-9.237500,1.200000])rotate([0,0,90.000000])m5(part,hole,block,casetop); // U3 (back)
};
module part_R3(part=true,hole=false,block=false)
{
translate([8.500000,-6.750000,1.200000])rotate([0,0,90.000000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_R7(part=true,hole=false,block=false)
{
translate([-5.000000,-8.150000,1.200000])rotate([0,0,-90.000000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_D2(part=true,hole=false,block=false)
{
translate([5.500000,-6.300000,1.200000])rotate([0,0,180.000000])m6(part,hole,block,casetop); // D2 (back)
};
module part_U4(part=true,hole=false,block=false)
{
translate([-0.600000,-10.000000,1.200000])rotate([0,0,180.000000])m5(part,hole,block,casetop); // U3 (back)
};
module part_J2(part=true,hole=false,block=false)
{
translate([13.800000,6.800000,1.200000])rotate([0,0,90.000000])translate([0.000000,-2.400000,0.000000])rotate([90.000000,-0.000000,-0.000000])m7(part,hole,block,casetop); // RevK:USB-C-Socket-H CSP-USC16-TR (back)
};
module part_U1(part=true,hole=false,block=false)
{
translate([3.500000,2.500000,1.200000])m8(part,hole,block,casetop); // U1 (back)
};
module part_Q3(part=true,hole=false,block=false)
{
translate([9.550000,-9.562500,1.200000])rotate([0,0,90.000000])m9(part,hole,block,casetop); // Q3 (back)
};
module part_R4(part=true,hole=false,block=false)
{
translate([-0.200000,-8.000000,1.200000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_V2(part=true,hole=false,block=false)
{
};
module part_C1(part=true,hole=false,block=false)
{
translate([-3.500000,-10.000000,1.200000])rotate([0,0,90.000000])m4(part,hole,block,casetop); // RevK:C_0805 C_0805_2012Metric (back)
};
module part_D3(part=true,hole=false,block=false)
{
translate([-13.000000,2.000000,1.200000])rotate([0,0,180.000000])m0(part,hole,block,casetop); // D1 (back)
};
module part_R5(part=true,hole=false,block=false)
{
translate([2.500000,-6.000000,1.200000])rotate([0,0,90.000000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_R9(part=true,hole=false,block=false)
{
translate([-5.000000,-6.250000,1.200000])rotate([0,0,-90.000000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_C5(part=true,hole=false,block=false)
{
translate([-5.000000,9.000000,1.200000])rotate([0,0,90.000000])m10(part,hole,block,casetop); // RevK:C_0402 C_0402_1005Metric (back)
};
module part_C20(part=true,hole=false,block=false)
{
translate([-3.000000,-6.000000,1.200000])rotate([0,0,180.000000])m11(part,hole,block,casetop); // RevK:C_0603 C_0603_1608Metric (back)
};
module part_Q1(part=true,hole=false,block=false)
{
translate([6.500000,-9.300000,1.200000])rotate([0,0,90.000000])m9(part,hole,block,casetop); // Q3 (back)
};
module part_V4(part=true,hole=false,block=false)
{
};
module part_R2(part=true,hole=false,block=false)
{
translate([9.500000,-6.750000,1.200000])rotate([0,0,-90.000000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_R6(part=true,hole=false,block=false)
{
translate([-15.000000,2.000000,1.200000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_J1(part=true,hole=false,block=false)
{
translate([15.000000,-7.000000,1.200000])rotate([0,0,90.000000])m12(part,hole,block,casetop,02); // J1 (back)
};
module part_R11(part=true,hole=false,block=false)
{
translate([12.000000,-9.500000,1.200000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
module part_D4(part=true,hole=false,block=false)
{
translate([-18.500000,2.000000,1.200000])m13(part,hole,block,casetop); // D4 (back)
};
module part_C19(part=true,hole=false,block=false)
{
translate([-5.000000,5.000000,1.200000])rotate([0,0,-90.000000])m10(part,hole,block,casetop); // RevK:C_0402 C_0402_1005Metric (back)
};
module part_J5(part=true,hole=false,block=false)
{
translate([-12.500000,7.000000,1.200000])m14(part,hole,block,casetop); // J5 (back)
};
module part_R8(part=true,hole=false,block=false)
{
translate([-5.000000,-9.950000,1.200000])rotate([0,0,-90.000000])m2(part,hole,block,casetop); // RevK:R_0402 R_0402_1005Metric (back)
};
// Parts to go on PCB (top)
module parts_top(part=false,hole=false,block=false){
part_D1(part,hole,block);
part_J3(part,hole,block);
part_R1(part,hole,block);
part_C2(part,hole,block);
part_D20(part,hole,block);
part_C26(part,hole,block);
part_V1(part,hole,block);
part_U3(part,hole,block);
part_R3(part,hole,block);
part_R7(part,hole,block);
part_D2(part,hole,block);
part_U4(part,hole,block);
part_J2(part,hole,block);
part_U1(part,hole,block);
part_Q3(part,hole,block);
part_R4(part,hole,block);
part_V2(part,hole,block);
part_C1(part,hole,block);
part_D3(part,hole,block);
part_R5(part,hole,block);
part_R9(part,hole,block);
part_C5(part,hole,block);
part_C20(part,hole,block);
part_Q1(part,hole,block);
part_V4(part,hole,block);
part_R2(part,hole,block);
part_R6(part,hole,block);
part_J1(part,hole,block);
part_R11(part,hole,block);
part_D4(part,hole,block);
part_C19(part,hole,block);
part_J5(part,hole,block);
part_R8(part,hole,block);
}

parts_top=14;
module part_J7(part=true,hole=false,block=false)
{
};
module part_PCB1(part=true,hole=false,block=false)
{
};
module part_J4(part=true,hole=false,block=false)
{
};
// Parts to go on PCB (bottom)
module parts_bottom(part=false,hole=false,block=false){
part_J7(part,hole,block);
part_PCB1(part,hole,block);
part_J4(part,hole,block);
}

parts_bottom=0;
module b(cx,cy,z,w,l,h){translate([cx-w/2,cy-l/2,z])cube([w,l,h]);}
module m0(part=false,hole=false,block=false,height)
{ // D1
// DFN1006-2L
if(part)
{
	b(0,0,0,1.0,0.6,0.45); // Chip
}
}

module m1(part=false,hole=false,block=false,height)
{ // J3
// WAGO-2060-45x-998-404
N=3;
if(part)
{
	translate([-0.9,0,0])hull()
	{
		b(0,0,0,12.7,N*4-0.1,1);
		translate([0.8,0,0])b(0,0,0,11.1,N*4-0.1,4.5);
	}
	for(p=[0:N-1])hull()
    {
        translate([-6,-4*(N-1)/2+p*4,2])sphere(d=3,$fn=12);
        translate([-11.510,0,2])sphere(d=4,$fn=12);
    }
}
if(hole)
{
    hull()
    {
        translate([-11.510,0,2])sphere(d=4,$fn=12);
        translate([-11.510,0,-height])sphere(d=4,$fn=12);
    }
    hull()
    {
        translate([-11.510,0,2])sphere(d=4,$fn=12);
        translate([-50,0,2])sphere(d=4,$fn=12);
    }
}
}

module m2(part=false,hole=false,block=false,height)
{ // RevK:R_0402 R_0402_1005Metric
// 0402 Resistor
if(part)
{
	b(0,0,0,1.5,0.65,0.2); // Pad size
	b(0,0,0,1.0,0.5,0.5); // Chip
}
}

module m3(part=false,hole=false,block=false,height)
{ // C2
if(part)
{
	b(0,0,0,7.5,4.5,3.1);
}
}

module m4(part=false,hole=false,block=false,height)
{ // RevK:C_0805 C_0805_2012Metric
// 0805 Capacitor
if(part)
{
	b(0,0,0,2,1.2,1); // Chip
	b(0,0,0,2,1.45,0.2); // Pad size
}
}

module m5(part=false,hole=false,block=false,height)
{ // U3
// SOT-23-5
if(part)
{
	b(0,0,0,1.726,3.026,1.2); // Part
	b(0,0,0,3.6,2.5,0.5); // Pins (well, one extra)
}
}

module m6(part=false,hole=false,block=false,height)
{ // D2
// SOD-123 Diode
if(part)
{
	b(0,0,0,2.85,1.8,1.35); // part
	b(0,0,0,4.2,1.2,0.7); // pads
}
}

module m7(part=false,hole=false,block=false,height)
{ // RevK:USB-C-Socket-H CSP-USC16-TR
// USB connector
rotate([-90,0,0])translate([-4.47,-3.84,0])
{
	if(part)
	{
		b(4.47,7,0,7,2,0.2);	// Pads
		translate([1.63,-0.2,1.63])
		rotate([-90,0,0])
		hull()
		{
			cylinder(d=3.26,h=7.55,$fn=24);
			translate([5.68,0,0])
			cylinder(d=3.26,h=7.55,$fn=24);
		}
		translate([0,6.2501,0])cube([8.94,1.1,1.6301]);
		translate([0,1.7,0])cube([8.94,1.6,1.6301]);
	}
	if(hole)
		translate([1.63,-20,1.63])
		rotate([-90,0,0])
	{
		// Plug
		hull()
		{
			cylinder(d=2.5,h=21,$fn=24);
			translate([5.68,0,0])
			cylinder(d=2.5,h=21,$fn=24);
		}
		hull()
		{
			cylinder(d=7,h=21,$fn=24);
			translate([5.68,0,0])
			cylinder(d=7,h=21,$fn=24);
		}
		translate([2.84,0,-100])
			cylinder(d=5,h=100,$fn=24);
	}
}
}

module m8(part=false,hole=false,block=false,height)
{ // U1
// ESP32-S3-MINI-1
translate([-15.4/2,-15.45/2,0])
{
	if(part)
	{
		cube([15.4,20.5,0.8]);
		translate([0.7,0.5,0])cube([14,13.55,2.4]);
	}
	if(hole)
	{
		cube([15.4,20.5,0.8]);
	}
}
}

module m9(part=false,hole=false,block=false,height)
{ // Q3
// SOT-23
if(part)
{
	b(0,0,0,1.4,3.0,1.1); // Body
	b(-0.9375,-0.95,0,1.475,0.6,0.5); // Pad
	b(-0.9375,0.95,0,1.475,0.6,0.5); // Pad
	b(0.9375,0,0,1.475,0.6,0.5); // Pad
}
}

module m10(part=false,hole=false,block=false,height)
{ // RevK:C_0402 C_0402_1005Metric
// 0402 Capacitor
if(part)
{
	b(0,0,0,1.0,0.5,1); // Chip
	b(0,0,0,1.5,0.65,0.2); // Pad size
}
}

module m11(part=false,hole=false,block=false,height)
{ // RevK:C_0603 C_0603_1608Metric
// 0603 Capacitor
if(part)
{
	b(0,0,0,1.6,0.8,1); // Chip
	b(0,0,0,1.6,0.95,0.2); // Pad size
}
}

module m12(part=false,hole=false,block=false,height,N=0)
{ // J1
translate([0,-4.5,0])rotate([90,0,0])
{
	if(part)
	{
		b(0,3,-6,3.9+2*N,6,6); // Body
		b(-1.55-N,2,-7.6,0.8,4,1.6); // Tag
		b(1.55+N,2,-7.6,0.8,4,1.6); // Tag
		for(n=[0:N-1])b(-1+n*2,0.25,-9.5,1,0.5,3.5); // pins
	}
	if(hole)
	{
		b(0,3,0,1.3+N*2,4,10);
	}
}
}

module m13(part=false,hole=false,block=false,height)
{ // D4
// 1x1mm LED
if(part)
{
        b(0,0,0,1.2,1.2,.8);
}
if(hole)
{
        hull()
        {
                b(0,0,.8,1.2,1.2,1);
                translate([0,0,height])cylinder(d=2,h=1,$fn=16);
        }
}
if(block)
{
        hull()
        {
                b(0,0,0,2.4,2.4,1);
                translate([0,0,height])cylinder(d=4,h=1,$fn=16);
        }
}
}

module m14(part=false,hole=false,block=false,height)
{ // J5
// WAGO-2060-45x-998-404
N=2;
if(part)
{
	translate([-0.9,0,0])hull()
	{
		b(0,0,0,12.7,N*4-0.1,1);
		translate([0.8,0,0])b(0,0,0,11.1,N*4-0.1,4.5);
	}
	for(p=[0:N-1])hull()
    {
        translate([-6,-4*(N-1)/2+p*4,2])sphere(d=3,$fn=12);
        translate([-11.510,0,2])sphere(d=4,$fn=12);
    }
}
if(hole)
{
    hull()
    {
        translate([-11.510,0,2])sphere(d=4,$fn=12);
        translate([-11.510,0,-height])sphere(d=4,$fn=12);
    }
    hull()
    {
        translate([-11.510,0,2])sphere(d=4,$fn=12);
        translate([-50,0,2])sphere(d=4,$fn=12);
    }
}
}

// Generate PCB casework

height=casebottom+pcbthickness+casetop;
$fn=48;

module pyramid()
{ // A pyramid
 polyhedron(points=[[0,0,0],[-height,-height,height],[-height,height,height],[height,height,height],[height,-height,height]],faces=[[0,1,2],[0,2,3],[0,3,4],[0,4,1],[4,3,2,1]]);
}


module pcb_hulled(h=pcbthickness,r=0)
{ // PCB shape for case
	if(useredge)outline(h,r);
	else hull()outline(h,r);
}

module solid_case(d=0)
{ // The case wall
	hull()
        {
                translate([0,0,-casebottom])pcb_hulled(height,casewall-edge);
                translate([0,0,edge-casebottom])pcb_hulled(height-edge*2,casewall);
        }
}

module preview()
{
	pcb();
	color("#0f0")parts_top(part=true);
	color("#0f0")parts_bottom(part=true);
	color("#f00")parts_top(hole=true);
	color("#f00")parts_bottom(hole=true);
	color("#00f8")parts_top(block=true);
	color("#00f8")parts_bottom(block=true);
}

module top_half(step=false)
{
	difference()
	{
		translate([-casebottom-100,-casewall-100,pcbthickness-lip/2+0.01]) cube([pcbwidth+casewall*2+200,pcblength+casewall*2+200,height]);
		if(step)translate([0,0,pcbthickness-lip/2-0.01])pcb_hulled(lip,casewall/2+fit);
	}
}

module bottom_half(step=false)
{
	translate([-casebottom-100,-casewall-100,pcbthickness+lip/2-height-0.01]) cube([pcbwidth+casewall*2+200,pcblength+casewall*2+200,height]);
	if(step)translate([0,0,pcbthickness-lip/2])pcb_hulled(lip,casewall/2-fit);
}

module case_wall()
{
	difference()
	{
		solid_case();
		translate([0,0,-height])pcb_hulled(height*2);
	}
}

module top_side_hole()
{
	difference()
	{
		intersection()
		{
			parts_top(hole=true);
			case_wall();
		}
		translate([0,0,-casebottom])pcb_hulled(height,casewall-edge);
	}
}

module bottom_side_hole()
{
	difference()
	{
		intersection()
		{
			parts_bottom(hole=true);
			case_wall();
		}
		translate([0,0,edge-casebottom])pcb_hulled(height-edge*2,casewall);
	}
}

module parts_space()
{
	minkowski()
	{
		union()
		{
			parts_top(part=true,hole=true);
			parts_bottom(part=true,hole=true);
		}
		sphere(r=margin,$fn=6);
	}
}

module top_cut()
{
	difference()
	{
		top_half(true);
		if(parts_top)difference()
		{
			minkowski()
			{ // Penetrating side holes
				top_side_hole();
				rotate([180,0,0])
				pyramid();
			}
			minkowski()
			{
				top_side_hole();
				rotate([0,0,45])cylinder(r=margin,h=height,$fn=4);
			}
		}
	}
	if(parts_bottom)difference()
	{
		minkowski()
		{ // Penetrating side holes
			bottom_side_hole();
			pyramid();
		}
			minkowski()
			{
				bottom_side_hole();
				rotate([0,0,45])translate([0,0,-height])cylinder(r=margin,h=height,$fn=4);
			}
	}
}

module bottom_cut()
{
	difference()
	{
		 translate([-casebottom-50,-casewall-50,-height]) cube([pcbwidth+casewall*2+100,pcblength+casewall*2+100,height*2]);
		 top_cut();
	}
}

module top_body()
{
	difference()
	{
		intersection()
		{
			solid_case();
			pcb_hulled(height);
			top_half();
		}
		if(parts_top)minkowski()
		{
			if(nohull)parts_top(part=true);
			else hull()parts_top(part=true);
			translate([0,0,margin-height])cylinder(r=margin,h=height,$fn=8);
		}
	}
	intersection()
	{
		solid_case();
		parts_top(block=true);
	}
}

module top_edge()
{
	intersection()
	{
		case_wall();
		top_cut();
	}
}

module top_pos()
{ // Position for plotting bottom
	translate([casewall,casewall,pcbthickness+casetop])rotate([180,0,0])children();
}

module pcb_pos()
{	// Position PCB relative to base 
		translate([0,0,pcbthickness-height])children();
}

module top()
{
	top_pos()difference()
	{
		union()
		{
			top_body();
			top_edge();
		}
		parts_space();
		pcb_pos()pcb(height,r=margin);
	}
}

module bottom_body()
{ // Position for plotting top
	difference()
	{
		intersection()
		{
			solid_case();
			translate([0,0,-height])pcb_hulled(height);
			bottom_half();
		}
		if(parts_bottom)minkowski()
		{
			if(nohull)parts_bottom(part=true);
			else hull()parts_bottom(part=true);
			translate([0,0,-margin])cylinder(r=margin,h=height,$fn=8);
		}
	}
	intersection()
	{
		solid_case();
		parts_bottom(block=true);
	}
}

module bottom_edge()
{
	intersection()
	{
		case_wall();
		bottom_cut();
	}
}

module bottom_pos()
{
	translate([casewall,casewall,casebottom])children();
}

module bottom()
{
	bottom_pos()difference()
	{
		union()
		{
        		bottom_body();
        		bottom_edge();
		}
		parts_space();
		pcb(height,r=margin);
	}
}
bottom(); translate([spacing,0,0])top();
