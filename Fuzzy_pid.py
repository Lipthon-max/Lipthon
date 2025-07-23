class FuzzyPID:
    def __init__(self, base_kp=2.0,kp_min = 0.1,kp_max = 10, base_ki=0.5,ki_min = 0.0,ki_max =5.0, base_kd=0.2, kd_min =0.0,kd_max = 2.0,
    output_min=-500, output_max=500,e_min = -100,e_max=100,ec_min=-50,ec_max=50):
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.output = 0
        self.last_error = 0
        self.prev_error = 0
        self.kp_min = kp_min
        self.kp_max = kp_max
        self.ki_min = ki_min
        self.ki_max = ki_max
        self.kd_min = kd_min
        self.kd_max = kd_max
        self.base_kp = base_kp
        self.base_ki = base_ki
        self.base_kd = base_kd
        self.e_min = e_min
        self.e_max = e_max
        self.ec_min = ec_min
        self.ec_max = ec_max

        self.output_min = output_min
        self.output_max = output_max
        #增量pid对应论域中的值
        self.qdetail_kp = 0
        self.qdetail_ki = 0
        self.qdetail_kd = 0
        #self.qfuzzy_output = 0
        self.errorsum = 0

        self.num_area = 8
        #隶属值
        self.e_memship_values = [-3,-2,-1,0,1,2,3]
        self.ec_memship_values = [-3,-2,-1,0,1,2,3]
        self.kp_memship_values = [-3,-2,-1,0,1,2,3]
        self.ki_memship_values = [-3,-2,-1,0,1,2,3]
        self.kd_memship_values = [-3,-2,-1,0,1,2,3]
        self.fuzzyoutput_memship_values = [-3,-2,-1,0,1,2,3]
        #
        self.e_gradmemship = [0,0]
        self.ec_gradmemship = [0,0]
        self.e_grad_index = [0,0]
        self.ec_grad_index = [0,0]
        #
        self.KpgradSums = [0,0,0,0,0,0,0]
        self.KigradSums = [0,0,0,0,0,0,0]
        self.KdgradSums = [0,0,0,0,0,0,0]

        self.NB = -3
        self.NM = -2
        self.NS = -1
        self.ZO = 0
        self.PS = 1
        self.PM = 2
        self.PB = 3
        # kp 规则表
        self.Kp_rule_list = [
            [self.PB, self.PB, self.PM, self.PM, self.PS, self.ZO, self.ZO],
            [self.PB, self.PB, self.PM, self.PS, self.PS, self.ZO, self.NS],
            [self.PM, self.PM, self.PM, self.PS, self.ZO, self.NS, self.NS],
            [self.PM, self.PM, self.PS, self.ZO, self.NS, self.NM, self.NM],
            [self.PS, self.PS, self.ZO, self.NS, self.NS, self.NM, self.NM],
            [self.PS, self.ZO, self.NS, self.NM, self.NM, self.NM, self.NB],
            [self.ZO, self.ZO, self.NM, self.NM, self.NM, self.NB, self.NB]
        ]

        # ki 规则表
        self.Ki_rule_list = [
            [self.NB, self.NB, self.NM, self.NM, self.NS, self.ZO, self.ZO],
            [self.NB, self.NB, self.NM, self.NS, self.NS, self.ZO, self.ZO],
            [self.NB, self.NM, self.NS, self.NS, self.ZO, self.PS, self.PS],
            [self.NM, self.NM, self.NS, self.ZO, self.PS, self.PM, self.PM],
            [self.NM, self.NS, self.ZO, self.PS, self.PS, self.PM, self.PB],
            [self.ZO, self.ZO, self.PS, self.PS, self.PM, self.PB, self.PB],
            [self.ZO, self.ZO, self.PS, self.PM, self.PM, self.PB, self.PB]
        ]

        # kd 规则表
        self.Kd_rule_list = [
            [self.PS, self.NS, self.NB, self.NB, self.NB, self.NM, self.PS],
            [self.PS, self.NS, self.NB, self.NM, self.NM, self.NS, self.ZO],
            [self.ZO, self.NS, self.NM, self.NM, self.NS, self.NS, self.ZO],
            [self.ZO, self.NS, self.NS, self.NS, self.NS, self.NS, self.ZO],
            [self.ZO, self.ZO, self.ZO, self.ZO, self.ZO, self.ZO, self.ZO],
            [self.PB, self.NS, self.PS, self.PS, self.PS, self.PS, self.PB],
            [self.PB, self.PM, self.PM, self.PM, self.PS, self.PS, self.PB]
        ]
        self.Fuzzy_rule_list = [
            [self.PB, self.PB, self.PB, self.PB, self.PM, self.ZO, self.ZO],
            [self.PB, self.PB, self.PB, self.PM, self.PM, self.ZO, self.ZO],
            [self.PB, self.PM, self.PM, self.PS, self.ZO, self.NS, self.NM],
            [self.PM, self.PM, self.PS, self.ZO, self.NS, self.NM, self.NM],
            [self.PS, self.PS, self.ZO, self.NM, self.NM, self.NM, self.NB],
            [self.ZO, self.ZO, self.ZO, self.NM, self.NB, self.NB, self.NB],
            [self.ZO, self.NS, self.NB, self.NB, self.NB, self.NB, self.NB]
        ]

    def calc_membership(self,erro,erro_c):
        #计算输入误差e和ec的隶属度

        if erro>self.e_memship_values[0] and erro < self.e_memship_values[6]:
            for i in range(self.num_area-2):
                if erro>=self.e_memship_values[i]and erro<=self.e_memship_values[i+1]:
                    self.e_gradmemship[0] = -(erro-self.e_memship_values[i+1])/(self.e_memship_values[i+1]-self.e_memship_values[i])
                    self.e_gradmemship[1] = 1+(erro-self.e_memship_values[i+1])/(self.e_memship_values[i+1]-self.e_memship_values[i])
                    self.e_grad_index[0] = i
                    self.e_grad_index[1] = i+1
                    break
        else:
            if erro<= self.e_memship_values[0]:
                self.e_gradmemship[0] = 1
                self.e_gradmemship[1] = 0
                self.e_grad_index[0] = 0
                self.e_grad_index[1] = -1
            elif erro>= self.e_memship_values[6]:
                self.e_gradmemship[0] = 1
                self.e_gradmemship[1] = 0
                self.e_grad_index[0] = 6
                self.e_grad_index[1] = -1

        if erro_c > self.ec_memship_values[0] and erro_c < self.ec_memship_values[6]:

            for i in range(self.num_area - 2):
                if erro_c >= self.ec_memship_values[i] and erro_c <= self.ec_memship_values[i + 1]:

                    self.ec_gradmemship[0] = -(erro_c - self.ec_memship_values[i + 1]) / (self.ec_memship_values[i + 1] - self.ec_memship_values[i])
                    self.ec_gradmemship[1] = 1 + (erro_c - self.ec_memship_values[i + 1]) / (self.ec_memship_values[i + 1] - self.ec_memship_values[i])
                    self.ec_grad_index[0] = i
                    self.ec_grad_index[1] = i + 1
                    break
        else:
            if erro_c <= self.ec_memship_values[0]:
                self.ec_gradmemship[0] = 1
                self.ec_gradmemship[1] = 0
                self.ec_grad_index[0] = 0
                self.ec_grad_index[1] = -1
            elif erro_c >= self.ec_memship_values[6]:
                self.ec_gradmemship[0] = 1
                self.ec_gradmemship[1] = 0
                self.ec_grad_index[0] = 6
                self.ec_grad_index[1] = -1
    def GetSumGrad(self):
        self.KpgradSums = [0]*7
        self.KigradSums = [0]*7
        self.KdgradSums = [0]*7
        for i in range(2):
            if self.e_grad_index[i] == -1:
                continue
            for j in range(2):
                if self.ec_grad_index[i] == -1:
                    continue
                kp_value = self.Kp_rule_list[self.e_grad_index[i]][self.ec_grad_index[j]]
                ki_value = self.Ki_rule_list[self.e_grad_index[i]][self.ec_grad_index[j]]
                kd_value = self.Kd_rule_list[self.e_grad_index[i]][self.ec_grad_index[j]]
                kp_index = kp_value+3
                ki_index = ki_value+3
                kd_index = kd_value+3
                memship_product = self.e_gradmemship[i]*self.ec_gradmemship[j]
                self.KpgradSums[kp_index] += memship_product
                self.KigradSums[ki_index] += memship_product
                self.KdgradSums[kd_index] += memship_product
    def GetOut(self):
        kp_numerator = 0
        kp_denominator = 0
        ki_numerator = 0
        ki_denominator = 0
        kd_numerator = 0
        kd_denominator = 0
        for i in range(7):
            kp_value = self.kp_memship_values[i]
            ki_value = self.ki_memship_values[i]
            kd_value = self.kd_memship_values[i]
            kp_numerator += kp_value*self.KpgradSums[i]
            ki_numerator += ki_value*self.KigradSums[i]
            kd_numerator += kd_value*self.KdgradSums[i]
            kp_denominator+=self.KpgradSums[i]
            ki_denominator+=self.KigradSums[i]
            kd_denominator+=self.KdgradSums[i]
        self.qdetail_kp = kp_numerator/kp_denominator if kp_denominator !=0 else 0
        self.qdetail_ki = ki_numerator/ki_denominator if ki_denominator !=0 else 0
        self.qdetail_kd = kd_numerator/kd_denominator if kd_denominator !=0 else 0
    def normalize(self, value, min_val, max_val):
        if max_val == min_val:
            return 0.0
        qvalue = 6.0 * (value - min_val) / (max_val - min_val) - 3.0
        return max(-3.0, min(3.0, qvalue))
    def denormalize(self, qvalue, min_val, max_val):
        if max_val == min_val:
            return min_val
        value = (max_val - min_val) * (qvalue + 3) / 6.0 + min_val
        return max(min_val, min(max_val, value))
    def FuzzyPIDcontroller(self,setpoint,actual):
        error = setpoint-actual
        error_change = error-self.last_error
        self.prev_error = self.last_error
        self.last_error = error
        normalized_error = self.normalize(error,self.e_min,self.e_max)
        normalized_ec = self.normalize(error_change,self.ec_min,self.ec_max)
        self.calc_membership(normalized_error,normalized_ec)
        self.GetSumGrad()
        self.GetOut()
        kp_adj = self.denormalize(self.qdetail_kp,self.kp_min,self.kp_max)
        ki_adj = self.denormalize(self.qdetail_ki,self.ki_min,self.ki_max)
        kd_adj = self.denormalize(self.qdetail_kd,self.kd_min,self.kd_max)
        self.kp = self.base_kp+kp_adj
        self.ki = self.base_ki+ki_adj
        self.kd = self.base_kd+kd_adj
        increment = (self.kp*(error-self.last_error))+self.ki*error+self.kd*(error-2*self.last_error+self.prev_error)
        return increment
    def get_output(self,setpoint,actual,output):
        delta = self.FuzzyPIDcontroller(setpoint,actual)
        output_act=output+delta
        if output_act>self.output_max:
            output_act = self.output_max
        elif output_act<self.output_min:
            output_act = self.output_min
        return output_act
    def step(self, setpoint, actual):
        delta = self.FuzzyPIDcontroller(setpoint, actual)
        self.output += delta
        self.output = max(self.output_min, min(self.output_max, self.output))
        return self.output
