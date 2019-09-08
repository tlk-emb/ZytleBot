class clf_res{
public:
    int not_red;
    int red;
    clf_res(int nr, int r){
        not_red = nr;
        red = r;
    }
    clf_res(){}
    inline clf_res& operator+=(const clf_res& rhs){
        not_red += rhs.not_red;
        red += rhs.red;
        return *this;
    }
    const clf_res operator+(const clf_res& rhs) const;

private:

};

const clf_res clf_res::operator+(const clf_res& rhs) const
{
    clf_res tmp;
    tmp.not_red = not_red + rhs.not_red;
    tmp.red = red + rhs.red;
    return tmp;
}

template <typename T>
class RandomForestClassifier{
private:
    static clf_res dt0(T X[584]);
    static clf_res dt1(T X[584]);
    static clf_res dt2(T X[584]);
    static clf_res dt3(T X[584]);
    static clf_res dt4(T X[584]);
    static clf_res dt5(T X[584]);
    static clf_res dt6(T X[584]);
    static clf_res dt7(T X[584]);
    static clf_res dt8(T X[584]);
    static clf_res dt9(T X[584]);
    // static clf_res (*estimators[])(T*) = {dt0, dt1, dt2, dt3, dt4, dt5, dt6, dt7, dt8, dt9};

public:
    RandomForestClassifier() {}
    ~RandomForestClassifier(){}
    static clf_res predict_proba(T X[584]);
};

template <typename T>
clf_res RandomForestClassifier<T>::predict_proba(T X[584]){
  clf_res rst = clf_res(0, 0);
  const int estimator_num = 10;
  clf_res tmpres;
  tmpres += RandomForestClassifier<T>::dt0(X);
  tmpres += RandomForestClassifier<T>::dt1(X);
  tmpres += RandomForestClassifier<T>::dt2(X);
  tmpres += RandomForestClassifier<T>::dt3(X);
  tmpres += RandomForestClassifier<T>::dt4(X);
  tmpres += RandomForestClassifier<T>::dt5(X);
  tmpres += RandomForestClassifier<T>::dt6(X);
  tmpres += RandomForestClassifier<T>::dt7(X);
  tmpres += RandomForestClassifier<T>::dt8(X);
  tmpres += RandomForestClassifier<T>::dt9(X);
  rst.not_red += tmpres.not_red;
  rst.red += tmpres.red;
  return rst;
}

template <typename T>
clf_res RandomForestClassifier<T>::dt0(T X[584]){
  if (X[526] <= 54.22465907534518){
    if (X[77] <= 101.08051127327172){
      if (X[433] <= 6.266663312077451){
        if (X[353] <= 55.8527705728476){
          return clf_res(23,  0);
        }else{  // if X[353] > 55.8527705728476
          return clf_res(2, 4);
        }
      }else{  // if X[433] > 6.266663312077451
        if (X[455] <= 46.439766177510286){
          return clf_res(7, 0);
        }else{  // if X[455] > 46.439766177510286
          return clf_res( 0, 62);
        }
      }
    }else{  // if X[77] > 101.08051127327172
      if (X[87] <= 73.54133185948176){
        if (X[355] <= 119.54329723407093){
          return clf_res(46,  0);
        }else{  // if X[355] > 119.54329723407093
          if (X[384] <= 57.13561456891585){
            if (X[350] <= 150.61713549645512){
              return clf_res(3, 1);
            }else{  // if X[350] > 150.61713549645512
              return clf_res(5, 4);
            }
          }else{  // if X[384] > 57.13561456891585
            return clf_res(7, 0);
          }
        }
      }else{  // if X[87] > 73.54133185948176
        return clf_res( 0, 16);
      }
    }
  }else{  // if X[526] > 54.22465907534518
    if (X[36] <= 79.43178322320423){
      if (X[296] <= 138.0738312769558){
        if (X[515] <= -24.410580074038265){
          return clf_res(11,  1);
        }else{  // if X[515] > -24.410580074038265
          if (X[183] <= 70.93917508145616){
            return clf_res(1, 9);
          }else{  // if X[183] > 70.93917508145616
            return clf_res( 0, 21);
          }
        }
      }else{  // if X[296] > 138.0738312769558
        if (X[309] <= 148.11198411551345){
          return clf_res(4, 1);
        }else{  // if X[309] > 148.11198411551345
          return clf_res(6, 0);
        }
      }
    }else{  // if X[36] > 79.43178322320423
      if (X[460] <= 76.94106713891946){
        return clf_res(2, 3);
      }else{  // if X[460] > 76.94106713891946
        return clf_res( 0, 46);
      }
    }
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt1(T X[584]){
  if (X[182] <= 83.62784659634497){
    if (X[242] <= 127.20317303721193){
      if (X[107] <= 78.33694404360733){
        if (X[243] <= 2.8448549015565874){
          return clf_res(2, 4);
        }else{  // if X[243] > 2.8448549015565874
          if (X[378] <= 88.09146751773868){
            if (X[340] <= 50.73735597011084){
              return clf_res(63,  0);
            }else{  // if X[340] > 50.73735597011084
              if (X[371] <= 32.864438563657146){
                return clf_res(15,  0);
              }else{  // if X[371] > 32.864438563657146
                return clf_res(5, 3);
              }
            }
          }else{  // if X[378] > 88.09146751773868
            return clf_res(1, 7);
          }
        }
      }else{  // if X[107] > 78.33694404360733
        return clf_res( 0, 11);
      }
    }else{  // if X[242] > 127.20317303721193
      if (X[488] <= 138.3531411874652){
        if (X[10] <= 81.3485510220327){
          if (X[116] <= 66.37728466524712){
            if (X[294] <= 136.01704336911817){
              if (X[371] <= 72.19043106436858){
                if (X[52] <= 73.75373783722657){
                  return clf_res(0, 9);
                }else{  // if X[52] > 73.75373783722657
                  if (X[433] <= 9.629788366552013){
                    return clf_res(3, 1);
                  }else{  // if X[433] > 9.629788366552013
                    return clf_res(4, 7);
                  }
                }
              }else{  // if X[371] > 72.19043106436858
                return clf_res(0, 9);
              }
            }else{  // if X[294] > 136.01704336911817
              return clf_res(6, 3);
            }
          }else{  // if X[116] > 66.37728466524712
            return clf_res( 0, 47);
          }
        }else{  // if X[10] > 81.3485510220327
          if (X[472] <= -25.052266959159443){
            return clf_res(9, 1);
          }else{  // if X[472] > -25.052266959159443
            if (X[474] <= 107.610328498458){
              if (X[541] <= 41.45303245555127){
                return clf_res(1, 5);
              }else{  // if X[541] > 41.45303245555127
                return clf_res(5, 0);
              }
            }else{  // if X[474] > 107.610328498458
              return clf_res(0, 8);
            }
          }
        }
      }else{  // if X[488] > 138.3531411874652
        if (X[233] <= 29.000530627552603){
          return clf_res(3, 2);
        }else{  // if X[233] > 29.000530627552603
          if (X[159] <= 41.338144622533726){
            return clf_res(15,  0);
          }else{  // if X[159] > 41.338144622533726
            return clf_res(3, 1);
          }
        }
      }
    }
  }else{  // if X[182] > 83.62784659634497
    if (X[425] <= 2.5284766073944525){
      return clf_res(2, 3);
    }else{  // if X[425] > 2.5284766073944525
      return clf_res( 0, 27);
    }
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt2(T X[584]){
  if (X[579] <= 79.82146782190226){
    if (X[385] <= 81.4453359675725){
      if (X[63] <= 69.68888738599335){
        if (X[420] <= 101.40046529936956){
          if (X[5] <= 94.57374404479827){
            if (X[471] <= 13.285727915310485){
              return clf_res(11,  0);
            }else{  // if X[471] > 13.285727915310485
              return clf_res(7, 1);
            }
          }else{  // if X[5] > 94.57374404479827
            return clf_res(26,  0);
          }
        }else{  // if X[420] > 101.40046529936956
          return clf_res(2, 3);
        }
      }else{  // if X[63] > 69.68888738599335
        return clf_res(1, 4);
      }
    }else{  // if X[385] > 81.4453359675725
      if (X[106] <= -0.9515461186715086){
        if (X[542] <= 56.25283601027063){
          if (X[128] <= 109.28518516142894){
            return clf_res(7, 2);
          }else{  // if X[128] > 109.28518516142894
            return clf_res(4, 3);
          }
        }else{  // if X[542] > 56.25283601027063
          if (X[454] <= 89.77883004180892){
            return clf_res(7, 0);
          }else{  // if X[454] > 89.77883004180892
            if (X[372] <= 84.98773358390577){
              return clf_res( 0, 11);
            }else{  // if X[372] > 84.98773358390577
              return clf_res(2, 5);
            }
          }
        }
      }else{  // if X[106] > -0.9515461186715086
        if (X[430] <= 58.09959881920031){
          return clf_res(19,  0);
        }else{  // if X[430] > 58.09959881920031
          if (X[424] <= -18.931896430591625){
            return clf_res(4, 0);
          }else{  // if X[424] > -18.931896430591625
            if (X[308] <= 99.43765943177108){
              if (X[74] <= 21.344254261705856){
                return clf_res( 0, 16);
              }else{  // if X[74] > 21.344254261705856
                return clf_res(6, 0);
              }
            }else{  // if X[308] > 99.43765943177108
              if (X[477] <= 116.90750311405006){
                return clf_res(  0, 105);
              }else{  // if X[477] > 116.90750311405006
                if (X[578] <= 161.06483848878693){
                  return clf_res(4, 4);
                }else{  // if X[578] > 161.06483848878693
                  return clf_res(0, 9);
                }
              }
            }
          }
        }
      }
    }
  }else{  // if X[579] > 79.82146782190226
    return clf_res(22,  0);
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt3(T X[584]){
  if (X[170] <= 45.37315346077437){
    if (X[448] <= 82.48056044318822){
      if (X[324] <= 73.03471789245971){
        return clf_res(5, 2);
      }else{  // if X[324] > 73.03471789245971
        return clf_res(13,  0);
      }
    }else{  // if X[448] > 82.48056044318822
      if (X[79] <= 3.1471691306824283){
        if (X[401] <= 39.32815779459428){
          return clf_res(10,  0);
        }else{  // if X[401] > 39.32815779459428
          if (X[498] <= 34.0195755443754){
            return clf_res(2, 2);
          }else{  // if X[498] > 34.0195755443754
            return clf_res( 0, 11);
          }
        }
      }else{  // if X[79] > 3.1471691306824283
        if (X[240] <= 69.68923099676992){
          if (X[346] <= 83.98083401286148){
            if (X[381] <= 109.38332313643158){
              return clf_res(5, 1);
            }else{  // if X[381] > 109.38332313643158
              if (X[376] <= 74.44783455657544){
                return clf_res(2, 4);
              }else{  // if X[376] > 74.44783455657544
                return clf_res( 0, 39);
              }
            }
          }else{  // if X[346] > 83.98083401286148
            return clf_res(4, 4);
          }
        }else{  // if X[240] > 69.68923099676992
          if (X[356] <= 114.40844236666783){
            return clf_res(2, 6);
          }else{  // if X[356] > 114.40844236666783
            return clf_res( 0, 24);
          }
        }
      }
    }
  }else{  // if X[170] > 45.37315346077437
    if (X[453] <= 93.27484392631195){
      return clf_res(83,  0);
    }else{  // if X[453] > 93.27484392631195
      if (X[484] <= 61.71003896029934){
        if (X[489] <= 78.43213857408588){
          return clf_res(4, 4);
        }else{  // if X[489] > 78.43213857408588
          if (X[254] <= 39.34514517186351){
            if (X[519] <= 100.96405201074116){
              return clf_res(4, 1);
            }else{  // if X[519] > 100.96405201074116
              return clf_res(1, 9);
            }
          }else{  // if X[254] > 39.34514517186351
            if (X[96] <= 87.21681351801931){
              return clf_res(1, 4);
            }else{  // if X[96] > 87.21681351801931
              return clf_res( 0, 33);
            }
          }
        }
      }else{  // if X[484] > 61.71003896029934
        return clf_res(4, 1);
      }
    }
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt4(T X[584]){
  if (X[453] <= 45.59997189525151){
    return clf_res(115,   0);
  }else{  // if X[453] > 45.59997189525151
    if (X[243] <= 131.91090985546438){
      if (X[85] <= 76.64107065902598){
        if (X[306] <= 95.40829809149635){
          if (X[35] <= 13.05348134383544){
            return clf_res( 0, 22);
          }else{  // if X[35] > 13.05348134383544
            return clf_res( 4, 10);
          }
        }else{  // if X[306] > 95.40829809149635
          if (X[445] <= 11.888841623534368){
            return clf_res(2, 6);
          }else{  // if X[445] > 11.888841623534368
            return clf_res( 0, 92);
          }
        }
      }else{  // if X[85] > 76.64107065902598
        if (X[506] <= 40.76910657232043){
          if (X[552] <= 68.76390065724469){
            return clf_res(0, 7);
          }else{  // if X[552] > 68.76390065724469
            return clf_res(3, 6);
          }
        }else{  // if X[506] > 40.76910657232043
          return clf_res(4, 1);
        }
      }
    }else{  // if X[243] > 131.91090985546438
      return clf_res(9, 4);
    }
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt5(T X[584]){
  if (X[170] <= 45.72444816020111){
    if (X[281] <= 112.99327750228511){
      if (X[133] <= 39.87068533900565){
        if (X[150] <= 69.75208060763502){
          if (X[462] <= 96.07825193263703){
            return clf_res(8, 0);
          }else{  // if X[462] > 96.07825193263703
            if (X[399] <= 52.70119340948129){
              return clf_res(0, 6);
            }else{  // if X[399] > 52.70119340948129
              return clf_res(5, 0);
            }
          }
        }else{  // if X[150] > 69.75208060763502
          return clf_res(1, 5);
        }
      }else{  // if X[133] > 39.87068533900565
        if (X[513] <= 123.65493838859548){
          return clf_res(25,  0);
        }else{  // if X[513] > 123.65493838859548
          return clf_res(2, 2);
        }
      }
    }else{  // if X[281] > 112.99327750228511
      if (X[379] <= 177.5781844936392){
        if (X[408] <= 76.38190879012143){
          if (X[150] <= 47.51956513420555){
            if (X[140] <= 70.09591231177288){
              return clf_res(0, 7);
            }else{  // if X[140] > 70.09591231177288
              return clf_res(2, 8);
            }
          }else{  // if X[150] > 47.51956513420555
            if (X[540] <= 126.03897373737456){
              return clf_res( 0, 62);
            }else{  // if X[540] > 126.03897373737456
              return clf_res(1, 7);
            }
          }
        }else{  // if X[408] > 76.38190879012143
          if (X[135] <= 99.72314724152835){
            return clf_res(4, 1);
          }else{  // if X[135] > 99.72314724152835
            return clf_res(8, 0);
          }
        }
      }else{  // if X[379] > 177.5781844936392
        return clf_res(4, 1);
      }
    }
  }else{  // if X[170] > 45.72444816020111
    if (X[519] <= -26.131271508100657){
      if (X[478] <= 45.899253589065715){
        return clf_res(23,  0);
      }else{  // if X[478] > 45.899253589065715
        return clf_res(6, 1);
      }
    }else{  // if X[519] > -26.131271508100657
      if (X[340] <= 67.10540373339032){
        if (X[50] <= 102.90722707356039){
          if (X[459] <= -14.170373339364236){
            return clf_res(5, 2);
          }else{  // if X[459] > -14.170373339364236
            if (X[251] <= 134.96443343543766){
              return clf_res(3, 3);
            }else{  // if X[251] > 134.96443343543766
              return clf_res( 0, 10);
            }
          }
        }else{  // if X[50] > 102.90722707356039
          if (X[116] <= 127.57207576812073){
            if (X[338] <= 17.498114517924556){
              return clf_res(30,  0);
            }else{  // if X[338] > 17.498114517924556
              return clf_res(11,  2);
            }
          }else{  // if X[116] > 127.57207576812073
            return clf_res( 0, 10);
          }
        }
      }else{  // if X[340] > 67.10540373339032
        if (X[412] <= 94.38001397004845){
          return clf_res( 0, 13);
        }else{  // if X[412] > 94.38001397004845
          return clf_res(3, 4);
        }
      }
    }
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt6(T X[584]){
  if (X[119] <= 62.45451780319108){
    if (X[407] <= 109.33717608467569){
      if (X[310] <= 71.97885439480163){
        return clf_res(88,  0);
      }else{  // if X[310] > 71.97885439480163
        if (X[36] <= 65.97961189750245){
          return clf_res(5, 1);
        }else{  // if X[36] > 65.97961189750245
          return clf_res(16,  0);
        }
      }
    }else{  // if X[407] > 109.33717608467569
      if (X[320] <= 180.91684936220088){
        if (X[329] <= 17.62254086210919){
          if (X[28] <= 72.08895819118696){
            return clf_res(3, 3);
          }else{  // if X[28] > 72.08895819118696
            return clf_res(5, 1);
          }
        }else{  // if X[329] > 17.62254086210919
          if (X[304] <= 93.18061560535101){
            return clf_res( 0, 88);
          }else{  // if X[304] > 93.18061560535101
            return clf_res(1, 7);
          }
        }
      }else{  // if X[320] > 180.91684936220088
        return clf_res(6, 1);
      }
    }
  }else{  // if X[119] > 62.45451780319108
    if (X[385] <= 136.75145460627223){
      if (X[443] <= 25.3210271813948){
        return clf_res(1, 4);
      }else{  // if X[443] > 25.3210271813948
        return clf_res( 0, 46);
      }
    }else{  // if X[385] > 136.75145460627223
      return clf_res(3, 6);
    }
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt7(T X[584]){
  if (X[359] <= 55.856743842293675){
    if (X[548] <= 139.27169648104297){
      if (X[443] <= 22.437463243982837){
        return clf_res(43,  0);
      }else{  // if X[443] > 22.437463243982837
        if (X[112] <= 98.07242128124484){
          return clf_res(5, 1);
        }else{  // if X[112] > 98.07242128124484
          return clf_res( 0, 28);
        }
      }
    }else{  // if X[548] > 139.27169648104297
      return clf_res(58,  0);
    }
  }else{  // if X[359] > 55.856743842293675
    if (X[90] <= -43.91594753801773){
      return clf_res(10,  0);
    }else{  // if X[90] > -43.91594753801773
      if (X[351] <= 159.66723332888893){
        if (X[246] <= 95.70279278478068){
          if (X[492] <= 76.34698143633833){
            if (X[113] <= 75.46243010937005){
              if (X[191] <= 5.141415393922596){
                return clf_res(0, 9);
              }else{  // if X[191] > 5.141415393922596
                return clf_res(2, 4);
              }
            }else{  // if X[113] > 75.46243010937005
              return clf_res( 0, 72);
            }
          }else{  // if X[492] > 76.34698143633833
            return clf_res(2, 6);
          }
        }else{  // if X[246] > 95.70279278478068
          if (X[95] <= 98.36196091500113){
            return clf_res(2, 4);
          }else{  // if X[95] > 98.36196091500113
            return clf_res(1, 4);
          }
        }
      }else{  // if X[351] > 159.66723332888893
        if (X[558] <= 154.64142224839026){
          if (X[141] <= -7.119521791673176){
            return clf_res(2, 5);
          }else{  // if X[141] > -7.119521791673176
            return clf_res(0, 9);
          }
        }else{  // if X[558] > 154.64142224839026
          if (X[190] <= 86.62845273743628){
            return clf_res(5, 7);
          }else{  // if X[190] > 86.62845273743628
            return clf_res(5, 1);
          }
        }
      }
    }
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt8(T X[584]){
  if (X[35] <= 44.25959322312327){
    if (X[559] <= 107.0405771728627){
      if (X[96] <= -33.37480122651332){
        return clf_res(9, 0);
      }else{  // if X[96] > -33.37480122651332
        if (X[360] <= 94.2885859548903){
          return clf_res(6, 2);
        }else{  // if X[360] > 94.2885859548903
          if (X[329] <= 56.29509718257399){
            if (X[31] <= 142.55530086859787){
              if (X[91] <= 40.24676566903631){
                return clf_res(8, 0);
              }else{  // if X[91] > 40.24676566903631
                return clf_res(3, 3);
              }
            }else{  // if X[31] > 142.55530086859787
              if (X[152] <= 13.081178750832866){
                if (X[349] <= 69.94500754253818){
                  return clf_res(2, 4);
                }else{  // if X[349] > 69.94500754253818
                  return clf_res(4, 5);
                }
              }else{  // if X[152] > 13.081178750832866
                return clf_res(0, 5);
              }
            }
          }else{  // if X[329] > 56.29509718257399
            if (X[539] <= 65.41684803375662){
              if (X[230] <= 53.233834294135875){
                return clf_res(0, 7);
              }else{  // if X[230] > 53.233834294135875
                return clf_res(1, 5);
              }
            }else{  // if X[539] > 65.41684803375662
              return clf_res( 0, 38);
            }
          }
        }
      }
    }else{  // if X[559] > 107.0405771728627
      if (X[234] <= 70.2743870718165){
        if (X[352] <= 50.600631742620735){
          return clf_res(0, 7);
        }else{  // if X[352] > 50.600631742620735
          if (X[564] <= 27.406974783377073){
            return clf_res(3, 1);
          }else{  // if X[564] > 27.406974783377073
            if (X[467] <= -19.26212043821811){
              return clf_res(2, 4);
            }else{  // if X[467] > -19.26212043821811
              if (X[188] <= 95.29022249964703){
                return clf_res(0, 5);
              }else{  // if X[188] > 95.29022249964703
                return clf_res(1, 3);
              }
            }
          }
        }
      }else{  // if X[234] > 70.2743870718165
        if (X[561] <= 93.39194194768334){
          return clf_res( 0, 20);
        }else{  // if X[561] > 93.39194194768334
          return clf_res(1, 7);
        }
      }
    }
  }else{  // if X[35] > 44.25959322312327
    if (X[391] <= 101.48239554683553){
      if (X[475] <= 11.089024434612664){
        return clf_res(38,  0);
      }else{  // if X[475] > 11.089024434612664
        if (X[68] <= 90.73931693318632){
          return clf_res(9, 2);
        }else{  // if X[68] > 90.73931693318632
          if (X[26] <= 61.897232889317316){
            return clf_res(13,  0);
          }else{  // if X[26] > 61.897232889317316
            return clf_res(5, 1);
          }
        }
      }
    }else{  // if X[391] > 101.48239554683553
      if (X[93] <= 42.899407054607){
        return clf_res(8, 5);
      }else{  // if X[93] > 42.899407054607
        return clf_res( 0, 48);
      }
    }
  }
}
template <typename T>
clf_res RandomForestClassifier<T>::dt9(T X[584]){
  if (X[551] <= 48.581830986538144){
    if (X[70] <= 142.59732704094733){
      if (X[381] <= -97.00487275706689){
        return clf_res(5, 1);
      }else{  // if X[381] > -97.00487275706689
        if (X[119] <= 86.25626744831928){
          if (X[507] <= 17.663615311625836){
            return clf_res(12,  0);
          }else{  // if X[507] > 17.663615311625836
            return clf_res(6, 1);
          }
        }else{  // if X[119] > 86.25626744831928
          return clf_res(81,  0);
        }
      }
    }else{  // if X[70] > 142.59732704094733
      if (X[111] <= 33.870489005929116){
        if (X[478] <= 88.75969785489526){
          return clf_res(6, 0);
        }else{  // if X[478] > 88.75969785489526
          return clf_res(1, 4);
        }
      }else{  // if X[111] > 33.870489005929116
        if (X[381] <= 164.3375691900038){
          return clf_res( 0, 19);
        }else{  // if X[381] > 164.3375691900038
          return clf_res(1, 4);
        }
      }
    }
  }else{  // if X[551] > 48.581830986538144
    if (X[18] <= 33.62020926819898){
      return clf_res(7, 0);
    }else{  // if X[18] > 33.62020926819898
      if (X[366] <= 102.10090062258223){
        if (X[232] <= 36.75467522311415){
          if (X[99] <= 18.692386455039127){
            return clf_res(1, 7);
          }else{  // if X[99] > 18.692386455039127
            return clf_res(6, 3);
          }
        }else{  // if X[232] > 36.75467522311415
          if (X[149] <= 6.699887444235358){
            if (X[16] <= 62.84255444893755){
              return clf_res(3, 2);
            }else{  // if X[16] > 62.84255444893755
              return clf_res(1, 3);
            }
          }else{  // if X[149] > 6.699887444235358
            if (X[16] <= 66.70585081478782){
              if (X[275] <= 163.31772253510528){
                return clf_res( 0, 75);
              }else{  // if X[275] > 163.31772253510528
                return clf_res( 1, 10);
              }
            }else{  // if X[16] > 66.70585081478782
              if (X[499] <= 85.60593042271049){
                return clf_res(3, 3);
              }else{  // if X[499] > 85.60593042271049
                if (X[323] <= 58.01198088674943){
                  return clf_res(2, 2);
                }else{  // if X[323] > 58.01198088674943
                  return clf_res(0, 9);
                }
              }
            }
          }
        }
      }else{  // if X[366] > 102.10090062258223
        return clf_res(4, 2);
      }
    }
  }
}
