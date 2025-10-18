    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 8;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (rtP)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtP.simstruct
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 27;
            section.data(27)  = dumData; %prealloc

                    ;% rtP.PIDController_D
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.PIDController_D_aybtd55nno
                    section.data(2).logicalSrcIdx = 2;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.PIDController_InitialConditionForFilter
                    section.data(3).logicalSrcIdx = 3;
                    section.data(3).dtTransOffset = 2;

                    ;% rtP.PIDController_InitialConditionForFilter_mxzkvf05nt
                    section.data(4).logicalSrcIdx = 4;
                    section.data(4).dtTransOffset = 3;

                    ;% rtP.LowPassFilterDiscreteorContinuous_K
                    section.data(5).logicalSrcIdx = 5;
                    section.data(5).dtTransOffset = 4;

                    ;% rtP.LowPassFilterDiscreteorContinuous2_K
                    section.data(6).logicalSrcIdx = 6;
                    section.data(6).dtTransOffset = 5;

                    ;% rtP.LowPassFilterDiscreteorContinuous1_K
                    section.data(7).logicalSrcIdx = 7;
                    section.data(7).dtTransOffset = 6;

                    ;% rtP.PIDController_N
                    section.data(8).logicalSrcIdx = 8;
                    section.data(8).dtTransOffset = 7;

                    ;% rtP.PIDController_N_mlojjm5ri3
                    section.data(9).logicalSrcIdx = 9;
                    section.data(9).dtTransOffset = 8;

                    ;% rtP.PIDController_P
                    section.data(10).logicalSrcIdx = 10;
                    section.data(10).dtTransOffset = 9;

                    ;% rtP.PIDController_P_iq3bdsbgdi
                    section.data(11).logicalSrcIdx = 11;
                    section.data(11).dtTransOffset = 10;

                    ;% rtP.LowPassFilterDiscreteorContinuous_T
                    section.data(12).logicalSrcIdx = 12;
                    section.data(12).dtTransOffset = 11;

                    ;% rtP.LowPassFilterDiscreteorContinuous2_T
                    section.data(13).logicalSrcIdx = 13;
                    section.data(13).dtTransOffset = 12;

                    ;% rtP.LowPassFilterDiscreteorContinuous1_T
                    section.data(14).logicalSrcIdx = 14;
                    section.data(14).dtTransOffset = 13;

                    ;% rtP.CompareToConstant_const
                    section.data(15).logicalSrcIdx = 15;
                    section.data(15).dtTransOffset = 14;

                    ;% rtP.CompareToConstant_const_ktpuubd0d5
                    section.data(16).logicalSrcIdx = 16;
                    section.data(16).dtTransOffset = 15;

                    ;% rtP.CompareToConstant_const_g0sgrhol0k
                    section.data(17).logicalSrcIdx = 17;
                    section.data(17).dtTransOffset = 16;

                    ;% rtP.CompareToConstant_const_coiiynw3vz
                    section.data(18).logicalSrcIdx = 18;
                    section.data(18).dtTransOffset = 17;

                    ;% rtP.CompareToConstant_const_jgfq3drtzc
                    section.data(19).logicalSrcIdx = 19;
                    section.data(19).dtTransOffset = 18;

                    ;% rtP.mmdistancesensors_debug
                    section.data(20).logicalSrcIdx = 20;
                    section.data(20).dtTransOffset = 19;

                    ;% rtP.LowPassFilterDiscreteorContinuous_init_option
                    section.data(21).logicalSrcIdx = 21;
                    section.data(21).dtTransOffset = 20;

                    ;% rtP.LowPassFilterDiscreteorContinuous1_init_option
                    section.data(22).logicalSrcIdx = 22;
                    section.data(22).dtTransOffset = 21;

                    ;% rtP.LowPassFilterDiscreteorContinuous2_init_option
                    section.data(23).logicalSrcIdx = 23;
                    section.data(23).dtTransOffset = 22;

                    ;% rtP.mmrobotpose_theta0
                    section.data(24).logicalSrcIdx = 24;
                    section.data(24).dtTransOffset = 23;

                    ;% rtP.mmwheelencoders_ticsperrev
                    section.data(25).logicalSrcIdx = 25;
                    section.data(25).dtTransOffset = 24;

                    ;% rtP.mmsimplerobot_x0
                    section.data(26).logicalSrcIdx = 26;
                    section.data(26).dtTransOffset = 25;

                    ;% rtP.mmsimplerobot_y0
                    section.data(27).logicalSrcIdx = 27;
                    section.data(27).dtTransOffset = 26;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtP.DetectFallNonpositive_vinit
                    section.data(1).logicalSrcIdx = 28;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(3) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtP.DetectRisePositive_vinit
                    section.data(1).logicalSrcIdx = 29;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.DetectRisePositive_vinit_lngnpu3jyt
                    section.data(2).logicalSrcIdx = 30;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            paramMap.sections(4) = section;
            clear section

            section.nData     = 44;
            section.data(44)  = dumData; %prealloc

                    ;% rtP.Out1_Y0
                    section.data(1).logicalSrcIdx = 31;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.Constant_Value
                    section.data(2).logicalSrcIdx = 32;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.Constant_Value_aig0pcsgad
                    section.data(3).logicalSrcIdx = 33;
                    section.data(3).dtTransOffset = 2;

                    ;% rtP.Constant_Value_lqgksk13vx
                    section.data(4).logicalSrcIdx = 34;
                    section.data(4).dtTransOffset = 3;

                    ;% rtP.Integrator_gainval
                    section.data(5).logicalSrcIdx = 35;
                    section.data(5).dtTransOffset = 4;

                    ;% rtP.Integrator_UpperSat
                    section.data(6).logicalSrcIdx = 36;
                    section.data(6).dtTransOffset = 5;

                    ;% rtP.Integrator_LowerSat
                    section.data(7).logicalSrcIdx = 37;
                    section.data(7).dtTransOffset = 6;

                    ;% rtP.Saturation_UpperSat
                    section.data(8).logicalSrcIdx = 38;
                    section.data(8).dtTransOffset = 7;

                    ;% rtP.Saturation_LowerSat
                    section.data(9).logicalSrcIdx = 39;
                    section.data(9).dtTransOffset = 8;

                    ;% rtP.Constant1_Value
                    section.data(10).logicalSrcIdx = 40;
                    section.data(10).dtTransOffset = 9;

                    ;% rtP.Integrator_gainval_oofl1tlpir
                    section.data(11).logicalSrcIdx = 41;
                    section.data(11).dtTransOffset = 10;

                    ;% rtP.Integrator_UpperSat_hfuo5t0da2
                    section.data(12).logicalSrcIdx = 42;
                    section.data(12).dtTransOffset = 11;

                    ;% rtP.Integrator_LowerSat_ovbjd4im3c
                    section.data(13).logicalSrcIdx = 43;
                    section.data(13).dtTransOffset = 12;

                    ;% rtP.Saturation_UpperSat_khw5hppl1l
                    section.data(14).logicalSrcIdx = 44;
                    section.data(14).dtTransOffset = 13;

                    ;% rtP.Saturation_LowerSat_f1bwbpmoly
                    section.data(15).logicalSrcIdx = 45;
                    section.data(15).dtTransOffset = 14;

                    ;% rtP.Integrator1_IC
                    section.data(16).logicalSrcIdx = 46;
                    section.data(16).dtTransOffset = 15;

                    ;% rtP.Integrator_IC
                    section.data(17).logicalSrcIdx = 47;
                    section.data(17).dtTransOffset = 16;

                    ;% rtP.Gain1_Gain
                    section.data(18).logicalSrcIdx = 48;
                    section.data(18).dtTransOffset = 17;

                    ;% rtP.Integrator_gainval_keeib2hddd
                    section.data(19).logicalSrcIdx = 49;
                    section.data(19).dtTransOffset = 18;

                    ;% rtP.Integrator_UpperSat_oshaxlda54
                    section.data(20).logicalSrcIdx = 50;
                    section.data(20).dtTransOffset = 19;

                    ;% rtP.Integrator_LowerSat_kruqrcyoju
                    section.data(21).logicalSrcIdx = 51;
                    section.data(21).dtTransOffset = 20;

                    ;% rtP.Saturation_UpperSat_c1owngef0q
                    section.data(22).logicalSrcIdx = 52;
                    section.data(22).dtTransOffset = 21;

                    ;% rtP.Saturation_LowerSat_llldnyvgbu
                    section.data(23).logicalSrcIdx = 53;
                    section.data(23).dtTransOffset = 22;

                    ;% rtP.Constant2_Value
                    section.data(24).logicalSrcIdx = 54;
                    section.data(24).dtTransOffset = 23;

                    ;% rtP.Integrator_IC_jmrokuovbr
                    section.data(25).logicalSrcIdx = 55;
                    section.data(25).dtTransOffset = 24;

                    ;% rtP.Gain_Gain
                    section.data(26).logicalSrcIdx = 56;
                    section.data(26).dtTransOffset = 25;

                    ;% rtP.Gain1_Gain_kyjkekkkxt
                    section.data(27).logicalSrcIdx = 57;
                    section.data(27).dtTransOffset = 26;

                    ;% rtP.Integrator_IC_o4vb0f3qhf
                    section.data(28).logicalSrcIdx = 58;
                    section.data(28).dtTransOffset = 27;

                    ;% rtP.Gain_Gain_lisflq1rpf
                    section.data(29).logicalSrcIdx = 59;
                    section.data(29).dtTransOffset = 28;

                    ;% rtP.Gain1_Gain_ha3bnp5l4j
                    section.data(30).logicalSrcIdx = 60;
                    section.data(30).dtTransOffset = 29;

                    ;% rtP.Integrator1_IC_pd3mj3kddd
                    section.data(31).logicalSrcIdx = 61;
                    section.data(31).dtTransOffset = 30;

                    ;% rtP.Gain_Gain_dlrejf44zs
                    section.data(32).logicalSrcIdx = 62;
                    section.data(32).dtTransOffset = 31;

                    ;% rtP.Gain1_Gain_bcmg3d0bsc
                    section.data(33).logicalSrcIdx = 63;
                    section.data(33).dtTransOffset = 32;

                    ;% rtP.Constant_Value_e2dunptcm0
                    section.data(34).logicalSrcIdx = 64;
                    section.data(34).dtTransOffset = 33;

                    ;% rtP.Constant_Value_b2pjk4fc2b
                    section.data(35).logicalSrcIdx = 65;
                    section.data(35).dtTransOffset = 34;

                    ;% rtP.Circumference_Value
                    section.data(36).logicalSrcIdx = 66;
                    section.data(36).dtTransOffset = 35;

                    ;% rtP.Circumference_Value_caan2y1dzp
                    section.data(37).logicalSrcIdx = 67;
                    section.data(37).dtTransOffset = 36;

                    ;% rtP.Motor_Left1_Value
                    section.data(38).logicalSrcIdx = 68;
                    section.data(38).dtTransOffset = 37;

                    ;% rtP.Motor_Right1_Value
                    section.data(39).logicalSrcIdx = 69;
                    section.data(39).dtTransOffset = 38;

                    ;% rtP.Constant_Value_e5jcotvvny
                    section.data(40).logicalSrcIdx = 70;
                    section.data(40).dtTransOffset = 39;

                    ;% rtP.Constant1_Value_ibpbochjmd
                    section.data(41).logicalSrcIdx = 71;
                    section.data(41).dtTransOffset = 40;

                    ;% rtP.Constant_Value_lr4ms5o4l0
                    section.data(42).logicalSrcIdx = 72;
                    section.data(42).dtTransOffset = 43;

                    ;% rtP.Constant1_Value_ddcl5gh2ak
                    section.data(43).logicalSrcIdx = 73;
                    section.data(43).dtTransOffset = 44;

                    ;% rtP.Constant2_Value_n2l4lxleww
                    section.data(44).logicalSrcIdx = 74;
                    section.data(44).dtTransOffset = 45;

            nTotData = nTotData + section.nData;
            paramMap.sections(5) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtP.OLED_STRING1_String
                    section.data(1).logicalSrcIdx = 75;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.OLED_STRING2_String
                    section.data(2).logicalSrcIdx = 76;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            paramMap.sections(6) = section;
            clear section

            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% rtP.Constant_Value_hfe5oy52it
                    section.data(1).logicalSrcIdx = 77;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.Constant_Value_leeph2khp1
                    section.data(2).logicalSrcIdx = 78;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.Constant_Value_hfn3nbsevr
                    section.data(3).logicalSrcIdx = 79;
                    section.data(3).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            paramMap.sections(7) = section;
            clear section

            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% rtP.Tick_per_rev_Gain
                    section.data(1).logicalSrcIdx = 80;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.Tick_per_rev_Gain_hhz0nuj2bt
                    section.data(2).logicalSrcIdx = 81;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.Delay_InitialCondition
                    section.data(3).logicalSrcIdx = 82;
                    section.data(3).dtTransOffset = 2;

                    ;% rtP.Delay_InitialCondition_h3211qidyt
                    section.data(4).logicalSrcIdx = 83;
                    section.data(4).dtTransOffset = 3;

            nTotData = nTotData + section.nData;
            paramMap.sections(8) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 7;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (rtB)
        ;%
            section.nData     = 53;
            section.data(53)  = dumData; %prealloc

                    ;% rtB.k0yntn5iai
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.oyebtlo0yf
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtB.lvavfjzz5q
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtB.llw2dpuhqu
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtB.m0rkmcd5js
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtB.ojwunabl10
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtB.n0opddonv2
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtB.osvcphwaz1
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 9;

                    ;% rtB.cq2zdm4bqp
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 12;

                    ;% rtB.gd41vhwken
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 14;

                    ;% rtB.jo4whguwls
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 17;

                    ;% rtB.itucdj1ewz
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 18;

                    ;% rtB.nvkrz3mxgj
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 19;

                    ;% rtB.l2snlaj1nc
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 21;

                    ;% rtB.idij42i2gw
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 22;

                    ;% rtB.dcpasfbtt4
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 23;

                    ;% rtB.hergh2yap5
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 25;

                    ;% rtB.n3zs4v3t2b
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 26;

                    ;% rtB.iqpya5s3hr
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 27;

                    ;% rtB.mvr2suztle
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 28;

                    ;% rtB.bg0bf32bte
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 30;

                    ;% rtB.l2ldfzarhu
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 31;

                    ;% rtB.p01dvdbqoi
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 32;

                    ;% rtB.i1mjatxjiw
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 33;

                    ;% rtB.a1of3lhpsj
                    section.data(25).logicalSrcIdx = 24;
                    section.data(25).dtTransOffset = 34;

                    ;% rtB.didojtsufe
                    section.data(26).logicalSrcIdx = 25;
                    section.data(26).dtTransOffset = 35;

                    ;% rtB.omofh2yahr
                    section.data(27).logicalSrcIdx = 26;
                    section.data(27).dtTransOffset = 36;

                    ;% rtB.bunjhnt5tq
                    section.data(28).logicalSrcIdx = 27;
                    section.data(28).dtTransOffset = 37;

                    ;% rtB.gwobx1gfe4
                    section.data(29).logicalSrcIdx = 28;
                    section.data(29).dtTransOffset = 38;

                    ;% rtB.ozsdm2k5rd
                    section.data(30).logicalSrcIdx = 29;
                    section.data(30).dtTransOffset = 39;

                    ;% rtB.j00txr32sr
                    section.data(31).logicalSrcIdx = 30;
                    section.data(31).dtTransOffset = 40;

                    ;% rtB.l1kjxwjqqg
                    section.data(32).logicalSrcIdx = 31;
                    section.data(32).dtTransOffset = 41;

                    ;% rtB.dbebchkcdj
                    section.data(33).logicalSrcIdx = 32;
                    section.data(33).dtTransOffset = 42;

                    ;% rtB.aiil34kpb4
                    section.data(34).logicalSrcIdx = 33;
                    section.data(34).dtTransOffset = 43;

                    ;% rtB.fnjiupqbnh
                    section.data(35).logicalSrcIdx = 34;
                    section.data(35).dtTransOffset = 44;

                    ;% rtB.mdqf3z5ief
                    section.data(36).logicalSrcIdx = 35;
                    section.data(36).dtTransOffset = 45;

                    ;% rtB.ay1hzknmrn
                    section.data(37).logicalSrcIdx = 36;
                    section.data(37).dtTransOffset = 46;

                    ;% rtB.lznx0xw0r2
                    section.data(38).logicalSrcIdx = 37;
                    section.data(38).dtTransOffset = 47;

                    ;% rtB.bcxqw2seh3
                    section.data(39).logicalSrcIdx = 38;
                    section.data(39).dtTransOffset = 48;

                    ;% rtB.njpg11ct4x
                    section.data(40).logicalSrcIdx = 39;
                    section.data(40).dtTransOffset = 49;

                    ;% rtB.ahfo3sk05y
                    section.data(41).logicalSrcIdx = 40;
                    section.data(41).dtTransOffset = 50;

                    ;% rtB.f3slecfhi1
                    section.data(42).logicalSrcIdx = 41;
                    section.data(42).dtTransOffset = 51;

                    ;% rtB.ez2cgqi4sl
                    section.data(43).logicalSrcIdx = 42;
                    section.data(43).dtTransOffset = 52;

                    ;% rtB.on1trmxucp
                    section.data(44).logicalSrcIdx = 43;
                    section.data(44).dtTransOffset = 53;

                    ;% rtB.nyluu4k4k5
                    section.data(45).logicalSrcIdx = 44;
                    section.data(45).dtTransOffset = 54;

                    ;% rtB.hvrs41ksah
                    section.data(46).logicalSrcIdx = 45;
                    section.data(46).dtTransOffset = 55;

                    ;% rtB.oi4ncurpey
                    section.data(47).logicalSrcIdx = 46;
                    section.data(47).dtTransOffset = 56;

                    ;% rtB.b5m2srzd22
                    section.data(48).logicalSrcIdx = 47;
                    section.data(48).dtTransOffset = 57;

                    ;% rtB.ofnpkggoia
                    section.data(49).logicalSrcIdx = 48;
                    section.data(49).dtTransOffset = 58;

                    ;% rtB.dvbjweik0a
                    section.data(50).logicalSrcIdx = 49;
                    section.data(50).dtTransOffset = 61;

                    ;% rtB.a3gdd2mdkb
                    section.data(51).logicalSrcIdx = 54;
                    section.data(51).dtTransOffset = 62;

                    ;% rtB.ocfwx3wy2w
                    section.data(52).logicalSrcIdx = 55;
                    section.data(52).dtTransOffset = 63;

                    ;% rtB.ktwu2bvg2d
                    section.data(53).logicalSrcIdx = 61;
                    section.data(53).dtTransOffset = 64;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section

            section.nData     = 10;
            section.data(10)  = dumData; %prealloc

                    ;% rtB.ob0lu21rke
                    section.data(1).logicalSrcIdx = 62;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.kiqq2yiad0
                    section.data(2).logicalSrcIdx = 63;
                    section.data(2).dtTransOffset = 1;

                    ;% rtB.awba5oiga4
                    section.data(3).logicalSrcIdx = 64;
                    section.data(3).dtTransOffset = 2;

                    ;% rtB.lcf41nfjxv
                    section.data(4).logicalSrcIdx = 65;
                    section.data(4).dtTransOffset = 3;

                    ;% rtB.ghfyexmhos
                    section.data(5).logicalSrcIdx = 66;
                    section.data(5).dtTransOffset = 4;

                    ;% rtB.hni21roo01
                    section.data(6).logicalSrcIdx = 67;
                    section.data(6).dtTransOffset = 5;

                    ;% rtB.ol2rqhhab3
                    section.data(7).logicalSrcIdx = 68;
                    section.data(7).dtTransOffset = 6;

                    ;% rtB.csw3govdsp
                    section.data(8).logicalSrcIdx = 69;
                    section.data(8).dtTransOffset = 7;

                    ;% rtB.jk11yckv5c
                    section.data(9).logicalSrcIdx = 70;
                    section.data(9).dtTransOffset = 8;

                    ;% rtB.ma1i1rk4i3
                    section.data(10).logicalSrcIdx = 71;
                    section.data(10).dtTransOffset = 9;

            nTotData = nTotData + section.nData;
            sigMap.sections(2) = section;
            clear section

            section.nData     = 12;
            section.data(12)  = dumData; %prealloc

                    ;% rtB.lhyjn2urf1
                    section.data(1).logicalSrcIdx = 72;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.p4hc2zjm4g
                    section.data(2).logicalSrcIdx = 73;
                    section.data(2).dtTransOffset = 1;

                    ;% rtB.d4qqji2cg3
                    section.data(3).logicalSrcIdx = 74;
                    section.data(3).dtTransOffset = 2;

                    ;% rtB.berhq5rp5u
                    section.data(4).logicalSrcIdx = 75;
                    section.data(4).dtTransOffset = 3;

                    ;% rtB.o5f5tw0bhw
                    section.data(5).logicalSrcIdx = 76;
                    section.data(5).dtTransOffset = 4;

                    ;% rtB.ajqs00jqrg
                    section.data(6).logicalSrcIdx = 77;
                    section.data(6).dtTransOffset = 5;

                    ;% rtB.dqjphyzewl
                    section.data(7).logicalSrcIdx = 78;
                    section.data(7).dtTransOffset = 6;

                    ;% rtB.oozws1hbtg
                    section.data(8).logicalSrcIdx = 79;
                    section.data(8).dtTransOffset = 7;

                    ;% rtB.f1tnuwwv30
                    section.data(9).logicalSrcIdx = 80;
                    section.data(9).dtTransOffset = 8;

                    ;% rtB.l3xv3oeegq
                    section.data(10).logicalSrcIdx = 81;
                    section.data(10).dtTransOffset = 9;

                    ;% rtB.dhnpill5yj
                    section.data(11).logicalSrcIdx = 82;
                    section.data(11).dtTransOffset = 10;

                    ;% rtB.heezp0izai
                    section.data(12).logicalSrcIdx = 83;
                    section.data(12).dtTransOffset = 11;

            nTotData = nTotData + section.nData;
            sigMap.sections(3) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtB.lmzy5tlewm.mbm0sbapgr
                    section.data(1).logicalSrcIdx = 84;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.lmzy5tlewm.eczzet5oy5
                    section.data(2).logicalSrcIdx = 85;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            sigMap.sections(4) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtB.mckzz5wvl4.mbm0sbapgr
                    section.data(1).logicalSrcIdx = 86;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.mckzz5wvl4.eczzet5oy5
                    section.data(2).logicalSrcIdx = 87;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            sigMap.sections(5) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtB.gg14xtouxc.mche153be2
                    section.data(1).logicalSrcIdx = 88;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(6) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtB.ovb0n2j4hj.mche153be2
                    section.data(1).logicalSrcIdx = 89;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(7) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 21;
        sectIdxOffset = 7;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (rtDW)
        ;%
            section.nData     = 39;
            section.data(39)  = dumData; %prealloc

                    ;% rtDW.a5mppswd4g
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.lmyjrqqm0q
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.cbu1x43z5o
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.dkwoq1zn2q
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.g4l4sstjgw
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.ix0bqloo3w
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.pfodavbtqb
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.cgcpzoopc3
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% rtDW.gyc2homrea
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% rtDW.gj0annlnte
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% rtDW.idqmwdd5yb
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% rtDW.buzua3mg0c
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% rtDW.njlzb3ymr4
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

                    ;% rtDW.lrjjgww0fr
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 13;

                    ;% rtDW.hvtlz3ytk4
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 14;

                    ;% rtDW.pwkg1i0clx
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 15;

                    ;% rtDW.ebtr03oego
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 16;

                    ;% rtDW.ihcygfgz50
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 17;

                    ;% rtDW.fgjijjwqop
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 18;

                    ;% rtDW.k0wn542ln0
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 19;

                    ;% rtDW.fxvkp0bdm5
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 20;

                    ;% rtDW.ou2xmqri5p
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 21;

                    ;% rtDW.j4tfuxf0r4
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 22;

                    ;% rtDW.a32ulgyecd
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 23;

                    ;% rtDW.keebpc1wve
                    section.data(25).logicalSrcIdx = 24;
                    section.data(25).dtTransOffset = 24;

                    ;% rtDW.bvrt110pwk
                    section.data(26).logicalSrcIdx = 25;
                    section.data(26).dtTransOffset = 25;

                    ;% rtDW.fi0ngahxjt
                    section.data(27).logicalSrcIdx = 26;
                    section.data(27).dtTransOffset = 26;

                    ;% rtDW.ok2ij41o2v
                    section.data(28).logicalSrcIdx = 27;
                    section.data(28).dtTransOffset = 27;

                    ;% rtDW.hs3ilvk2e4
                    section.data(29).logicalSrcIdx = 28;
                    section.data(29).dtTransOffset = 28;

                    ;% rtDW.kt54bbuvjk
                    section.data(30).logicalSrcIdx = 29;
                    section.data(30).dtTransOffset = 29;

                    ;% rtDW.ddopguiexo
                    section.data(31).logicalSrcIdx = 30;
                    section.data(31).dtTransOffset = 30;

                    ;% rtDW.nxxmufiqog
                    section.data(32).logicalSrcIdx = 31;
                    section.data(32).dtTransOffset = 31;

                    ;% rtDW.l010k3tsbk
                    section.data(33).logicalSrcIdx = 32;
                    section.data(33).dtTransOffset = 32;

                    ;% rtDW.ijbaarr4nz
                    section.data(34).logicalSrcIdx = 33;
                    section.data(34).dtTransOffset = 33;

                    ;% rtDW.o00jirvkiw
                    section.data(35).logicalSrcIdx = 34;
                    section.data(35).dtTransOffset = 34;

                    ;% rtDW.hdavbqdih1
                    section.data(36).logicalSrcIdx = 35;
                    section.data(36).dtTransOffset = 35;

                    ;% rtDW.kw1wkwwjxk
                    section.data(37).logicalSrcIdx = 36;
                    section.data(37).dtTransOffset = 36;

                    ;% rtDW.mcj31hnz02
                    section.data(38).logicalSrcIdx = 37;
                    section.data(38).dtTransOffset = 37;

                    ;% rtDW.ar1zrakxb3
                    section.data(39).logicalSrcIdx = 38;
                    section.data(39).dtTransOffset = 38;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 20;
            section.data(20)  = dumData; %prealloc

                    ;% rtDW.cfivhtuao0.LoggedData
                    section.data(1).logicalSrcIdx = 39;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.bb4jgoabcn.LoggedData
                    section.data(2).logicalSrcIdx = 40;
                    section.data(2).dtTransOffset = 2;

                    ;% rtDW.g5vm0pxz20.LoggedData
                    section.data(3).logicalSrcIdx = 41;
                    section.data(3).dtTransOffset = 3;

                    ;% rtDW.iro3gppa3o.LoggedData
                    section.data(4).logicalSrcIdx = 42;
                    section.data(4).dtTransOffset = 4;

                    ;% rtDW.ltnd253qsk.LoggedData
                    section.data(5).logicalSrcIdx = 43;
                    section.data(5).dtTransOffset = 5;

                    ;% rtDW.patxlot3wj.LoggedData
                    section.data(6).logicalSrcIdx = 44;
                    section.data(6).dtTransOffset = 6;

                    ;% rtDW.gdwkbvj3os.LoggedData
                    section.data(7).logicalSrcIdx = 45;
                    section.data(7).dtTransOffset = 7;

                    ;% rtDW.ikpzcg3ezr.LoggedData
                    section.data(8).logicalSrcIdx = 46;
                    section.data(8).dtTransOffset = 8;

                    ;% rtDW.a0wn5dv0oh.LoggedData
                    section.data(9).logicalSrcIdx = 47;
                    section.data(9).dtTransOffset = 9;

                    ;% rtDW.n5hawsku0t.LoggedData
                    section.data(10).logicalSrcIdx = 48;
                    section.data(10).dtTransOffset = 10;

                    ;% rtDW.f0eloybr32.LoggedData
                    section.data(11).logicalSrcIdx = 49;
                    section.data(11).dtTransOffset = 11;

                    ;% rtDW.iike22gcof.LoggedData
                    section.data(12).logicalSrcIdx = 50;
                    section.data(12).dtTransOffset = 12;

                    ;% rtDW.m0npouxp1d.LoggedData
                    section.data(13).logicalSrcIdx = 51;
                    section.data(13).dtTransOffset = 14;

                    ;% rtDW.ga3kjdz0ce.LoggedData
                    section.data(14).logicalSrcIdx = 52;
                    section.data(14).dtTransOffset = 15;

                    ;% rtDW.pr0y2sredb.LoggedData
                    section.data(15).logicalSrcIdx = 53;
                    section.data(15).dtTransOffset = 16;

                    ;% rtDW.b0gq1idaar.LoggedData
                    section.data(16).logicalSrcIdx = 54;
                    section.data(16).dtTransOffset = 17;

                    ;% rtDW.dh1m5auff4.LoggedData
                    section.data(17).logicalSrcIdx = 55;
                    section.data(17).dtTransOffset = 19;

                    ;% rtDW.kep231mdda.LoggedData
                    section.data(18).logicalSrcIdx = 56;
                    section.data(18).dtTransOffset = 20;

                    ;% rtDW.hzap5lnx0x.AQHandles
                    section.data(19).logicalSrcIdx = 57;
                    section.data(19).dtTransOffset = 21;

                    ;% rtDW.py1hl0uwoj.LoggedData
                    section.data(20).logicalSrcIdx = 58;
                    section.data(20).dtTransOffset = 22;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% rtDW.ectirthedq
                    section.data(1).logicalSrcIdx = 59;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.gfxrso0ugv
                    section.data(2).logicalSrcIdx = 60;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.nnwjuiot3s
                    section.data(3).logicalSrcIdx = 61;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.bd3pmtlbxa
                    section.data(4).logicalSrcIdx = 62;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.igyxtuvsol
                    section.data(5).logicalSrcIdx = 63;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.kja2yab2jc
                    section.data(6).logicalSrcIdx = 64;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.nssubxjuqy
                    section.data(7).logicalSrcIdx = 65;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.ou0eypazjv
                    section.data(8).logicalSrcIdx = 66;
                    section.data(8).dtTransOffset = 7;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% rtDW.d0bfvwdil1
                    section.data(1).logicalSrcIdx = 67;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.gkom5dg04h
                    section.data(2).logicalSrcIdx = 68;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.oqmdwhzvqd
                    section.data(3).logicalSrcIdx = 69;
                    section.data(3).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% rtDW.klyc1nbon0
                    section.data(1).logicalSrcIdx = 70;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.faxliy3dxh
                    section.data(2).logicalSrcIdx = 71;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.hc5a2ng0u1
                    section.data(3).logicalSrcIdx = 72;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.a2i0vc0nyr
                    section.data(4).logicalSrcIdx = 73;
                    section.data(4).dtTransOffset = 3;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.m2kqvebxm1
                    section.data(1).logicalSrcIdx = 74;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% rtDW.ozgxdva2cb
                    section.data(1).logicalSrcIdx = 75;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.l32ey2ufcp
                    section.data(2).logicalSrcIdx = 76;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.jlhwx40y1w
                    section.data(3).logicalSrcIdx = 77;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.doubgfhhla
                    section.data(4).logicalSrcIdx = 78;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.dfi5eeqqke
                    section.data(5).logicalSrcIdx = 79;
                    section.data(5).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(7) = section;
            clear section

            section.nData     = 6;
            section.data(6)  = dumData; %prealloc

                    ;% rtDW.gjcro2b4mx
                    section.data(1).logicalSrcIdx = 80;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.hynvkpd32l
                    section.data(2).logicalSrcIdx = 81;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.g2u1e0iaht
                    section.data(3).logicalSrcIdx = 82;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.kufypb5p4u
                    section.data(4).logicalSrcIdx = 83;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.csj4ylidjq
                    section.data(5).logicalSrcIdx = 84;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.dt2xtcrnce
                    section.data(6).logicalSrcIdx = 85;
                    section.data(6).dtTransOffset = 5;

            nTotData = nTotData + section.nData;
            dworkMap.sections(8) = section;
            clear section

            section.nData     = 7;
            section.data(7)  = dumData; %prealloc

                    ;% rtDW.kdcruf4nch
                    section.data(1).logicalSrcIdx = 86;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.ev1smh1ixm
                    section.data(2).logicalSrcIdx = 87;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.i0x4wmxvc4
                    section.data(3).logicalSrcIdx = 88;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.a4r3kbjevf
                    section.data(4).logicalSrcIdx = 89;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.jyntvb5nm4
                    section.data(5).logicalSrcIdx = 90;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.kego0fkvyv
                    section.data(6).logicalSrcIdx = 91;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.dehjsdofsv
                    section.data(7).logicalSrcIdx = 92;
                    section.data(7).dtTransOffset = 6;

            nTotData = nTotData + section.nData;
            dworkMap.sections(9) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.lmzy5tlewm.gpq1yop1nj
                    section.data(1).logicalSrcIdx = 93;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(10) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.lmzy5tlewm.h1velika3t
                    section.data(1).logicalSrcIdx = 94;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(11) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.lmzy5tlewm.fl0i5tniw0
                    section.data(1).logicalSrcIdx = 95;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(12) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.mckzz5wvl4.gpq1yop1nj
                    section.data(1).logicalSrcIdx = 96;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(13) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.mckzz5wvl4.h1velika3t
                    section.data(1).logicalSrcIdx = 97;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(14) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.mckzz5wvl4.fl0i5tniw0
                    section.data(1).logicalSrcIdx = 98;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(15) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.gg14xtouxc.jjh5jiiwwz
                    section.data(1).logicalSrcIdx = 99;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(16) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.gg14xtouxc.o2n3z045kg
                    section.data(1).logicalSrcIdx = 100;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(17) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.gg14xtouxc.d14swqzgdy
                    section.data(1).logicalSrcIdx = 101;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(18) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.ovb0n2j4hj.jjh5jiiwwz
                    section.data(1).logicalSrcIdx = 102;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(19) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.ovb0n2j4hj.o2n3z045kg
                    section.data(1).logicalSrcIdx = 103;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(20) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.ovb0n2j4hj.d14swqzgdy
                    section.data(1).logicalSrcIdx = 104;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(21) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 3171182202;
    targMap.checksum1 = 1425351409;
    targMap.checksum2 = 867518356;
    targMap.checksum3 = 3277463169;

