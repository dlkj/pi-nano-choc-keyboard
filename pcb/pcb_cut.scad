$fn=128;
keycap = [18, 17];
keyswitch = [15, 15]; //15,15
thumb_r = 80;
thumb_c = [-36, -165.5];


module keys(size) {
    //core keys that are part of the staggered grid

    column_offset=[0, 2, 9, 2, -9, -9];
    column_count=[4,4,4,4,5,5];

    for(j = [0:5] ) {
        for (i = [0 : column_count[j]-1] ) {
            translate([-keycap[0]*j, -keycap[1]*i+column_offset[j], 0]) square(size, center=true);
        }
    }

    //column 4 modifier
    translate([keycap[0]*-3, -keycap[1]*4+column_offset[4]]) {
        square(size, center=true);
    }


    //thumb keys
    translate(thumb_c){
        {
            translate([0,thumb_r ]) square(size, center=true);
            translate([0, thumb_r + keycap[1]]) square(size, center=true);
        }
        rotate(a=-15){
            translate([0,thumb_r ]) square(size, center=true);
            translate([0, thumb_r + keycap[1]]) square(size, center=true);
        }
        rotate(a=-30){
            translate([0,thumb_r ]) square(size, center=true);
            translate([0, thumb_r + keycap[1]]) square(size, center=true);
        }
        rotate(a=-45){
            translate([0,thumb_r ]) square(size, center=true);
            translate([0, thumb_r + keycap[1]]) square(size, center=true);
        }
    }
}

//alignment glyp
color("red") translate([30,30]){
    difference() {
        circle(5);
        circle(4.75);
    }
    square([10,.25],center = true);
    square([.25,10],center = true); 
}

difference(){
    minkowski() {
        circle(1);
        group() {
            //Fill center board holes
            translate([-72,-94]) square([97,94]);


            //pico
            color("green") translate([3, -52]) square([22, 52]);


            translate(-keycap/2) color("green"){
                outer_r = thumb_r + keycap[1]*1.5;
                inner_r = thumb_r - keycap[1]*.5;

                //Fill thumb cluster
                translate(thumb_c) {
                    difference(){
                        difference() {
                            circle(outer_r);
                            circle(inner_r);
                        }
                        {
                            translate([-outer_r,0]) square([outer_r*2, outer_r*2], center = true);
                            rotate(-45) translate([+outer_r,0]) square([outer_r*2, outer_r*2], center = true);
                        }
                    }
                }

                keys(keycap);
            }
        }
    }
    translate(-keycap/2) keys(keyswitch);
}

module fill_bl(){
    difference(){
        translate([-1,-1]) square([1,1]);
        circle(1);
    }
}

module fill_br(){
    difference(){
        translate([0,-1]) square([1,1]);
        circle(1);
    }
}

module fill_tr(){
    difference(){
        square([1,1]);
        circle(1);
    }
}

module fill_45(){
    difference(){
        polygon(points=[[0,0],[-1,0], [-2,-1],[-1,-1] ], paths=[[0,1,2,3]]);
        circle(1);
    }
}

//smooth concave corners
translate([-keycap[0]+2, 2]) {
    fill_bl();
}

translate([-keycap[0]*2+2, 4]) {
    fill_bl();
}

translate([-keycap[0]*3-2, 4]) {
    fill_br();
}

translate([-keycap[0]*4-2, -7]) {
    fill_br();
}

translate([-keycap[0]*3-2, -keycap[1]*6+6]) {
    fill_tr();
}

translate([27, -94]) {
    fill_45();
}