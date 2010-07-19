#! /usr/bin/env python
'''
Module that permits generating reports by reading properties files
'''

from __future__ import with_statement

import os
import sys
import shutil
import re
from optparse import OptionParser
from glob import glob
from collections import defaultdict
import logging
import datetime

logging.basicConfig(level=logging.INFO,
                    format='%(levelname)-8s %(message)s',)
                    
import tools
import planning_suites
from markup import Document
from external.configobj import ConfigObj
from external.datasets import DataSet
from external import txt2tags
from external import argparse

class ReportOptionParser(argparse.ArgumentParser):
    def __init__(self, *args, **kwargs):
        argparse.ArgumentParser.__init__(self, *args, 
                formatter_class=argparse.ArgumentDefaultsHelpFormatter, **kwargs)
      
        def directory(string):
            if not os.path.isdir(string):
                msg = "%r is not an evaluation directory" % string
                raise argparse.ArgumentTypeError(msg)
            return string
        
      
        self.add_argument('source', help='path to evaluation directory', type=directory)
        
        self.add_argument('-d', '--dest', dest='report_dir',
                        help='path to report directory',
                        #default='report_%s' % datetime.datetime.now().isoformat(),
                        default='reports'
                        )
                        
        self.add_argument('--format', dest='output_format', default='tex',
                            help='format of the output file',
                            choices=sorted(txt2tags.TARGETS))
                            
        self.add_argument('focus', 
                            help='the analyzed attribute (e.g. "expanded")')
                        
    def parse_args(self):
        args = argparse.ArgumentParser.parse_args(self)
        args.eval_dir = args.source
        
        args.eval_dir = os.path.normpath(os.path.abspath(args.eval_dir))
        logging.info('Eval dir: "%s"' % args.eval_dir)
        
        if not os.path.exists(args.report_dir):
            os.makedirs(args.report_dir)
        
        #if not args.report_dir:
        #    parent_dir = os.path.dirname(args.eval_dir)
        #    dir_name = os.path.basename(args.eval_dir)
        #    args.report_dir = os.path.join(parent_dir, dir_name + '-report')
        #    logging.info('Report dir: "%s"' % args.report_dir)
        
        return args
        
    

class Report(object):
    '''
    Base class for all reports
    '''
    
    def __init__(self, parser=ReportOptionParser()):
        self.parser = parser
        args = self.parser.parse_args()
        
        # Give all the options to the report instance
        self.__dict__.update(args.__dict__)
        
        self.data = self._get_data()
        
        self.filter_funcs = []
        self.filter_pairs = {}
        
        self.grouping = []
        self.order = []

        
    def set_focus(self, attribute):
        '''
        Define which attribute the report should be about
        '''
        self.focus = attribute
        
        
    def add_filter(self, *filter_funcs, **filter_pairs):
        self.filter_funcs.extend(filter_funcs)
        self.filter_pairs.update(filter_pairs)
        
    def set_grouping(self, *grouping):
        '''
        Set by which attributes the runs should be separated into groups
        
        grouping = None/[]: Use only one big group (default)
        grouping = 'domain': group by domain
        grouping = ['domain', 'problem']: Use one group for each problem
        '''
        self.grouping = grouping
        
    def set_order(self, *order):
        self.order = order
        
    def _get_group_dict(self):
        data = DataSet(self.data)
        data.dump()
        
        if not self.order:
            self.order = ['id']
        data.sort(*self.order)
        print 'SORTED'
        data.dump()
        
        if self.filter_funcs or self.filter_pairs:
            data = data.filtered(*self.filter_funcs, **self.filter_pairs)
            print 'FILTERED'
            data.dump()
        
        group_dict = data.group_dict(*self.grouping)
        print 'GROUPED'
        print group_dict
                
        return group_dict
        
        
    def _get_data(self):
        data = DataSet()
        for base, dir, files in os.walk(self.eval_dir):
            for file in files:
                if file == 'properties':
                    file = os.path.join(base, file)
                    props = tools.Properties(file)
                    data.append(**props)
        return data
    
    
    def _get_table():
        pass
        

class AbsolutePlanningReportOptionParser(ReportOptionParser):
    def __init__(self, *args, **kwargs):
        ReportOptionParser.__init__(self, *args, **kwargs)
        
        self.add_argument('-c', '--configs', nargs='+', required=True, 
                            help="planner configurations")
            
        self.add_argument('-s', '--suite', nargs='+', required=True, 
                            help='tasks, domains or suites')
            
        self.add_argument('-r', '--resolution', default='domain',
                            choices=['suite', 'domain', 'problem'])
                            
                            
    def parse_args(self):
        args = ReportOptionParser.parse_args(self)
        print 'ARGUMENTS:', args
        
        return args


class AbsolutePlanningReport(Report):
    '''
    '''
    def __init__(self, *args, **kwargs):
        Report.__init__(self, AbsolutePlanningReportOptionParser(), *args, **kwargs)
        
        self.output = ''
        
        self.problems = planning_suites.build_suite(self.suite)
        
        def filter_by_problem(run):
            for problem in self.problems:
                if problem.domain == run['domain'] and problem.problem == run['problem']:
                    return True
            return False
            
        def filter_by_config(run):
            for config in self.configs:
                if config == run['config']:
                    return True
            return False
        
        self.add_filter(filter_by_problem, filter_by_config)
        
        if self.resolution == 'suite':
            self.set_grouping('config')
        elif self.resolution == 'domain':
            self.set_grouping('config', 'domain')
        elif self.resolution == 'problem':
            self.set_grouping('config', 'domain', 'problem')
            
            
    @property
    def name(self):
        parts = [self.configs, self.suite, ['by-'+self.resolution], [self.focus]]
        return '_'.join(['-'.join(vars) for vars in parts])
        
        
    def build(self):
        if not self.focus:
            self.parser.error('You have to set the focus attribute')
        
        group_dict = self._get_group_dict()
        
        table = {}
        
        if self.resolution == 'suite':
            for (config,), group in group_dict.items():
                table[('-'.join(self.suite), config)] = sum(group[self.focus])
        elif self.resolution == 'domain':
            for (config, domain), group in group_dict.items():
                table[(domain, config)] = sum(group[self.focus])
        elif self.resolution == 'problem':
            for (config, domain, problem), group in group_dict.items():
                table[(domain + ':' + problem, config)] = sum(group[self.focus])
            
        print 'TABLE'
        print table
        
        doc = Document(title=self.name)
        doc.add_table(table, title=self.focus)
        print 'TEXT:'
        print doc.text
        self.output = doc.render(self.output_format)
        print 'OUTPUT', self.output
        return self.output
        
        
    def write(self):
        if not self.output:
            self.output = self.build()
        
        output_file = os.path.join(self.report_dir, 
            self.name + '.' + self.output_format)
        with open(output_file, 'w') as file:
            logging.info('Writing output to "%s"' % output_file)
            file.write(self.output)
        


if __name__ == "__main__":
    if len(sys.argv) == 1:
        sys.argv.extend('test-eval expanded -s MINITEST -c yY'.split())
    
    report = AbsolutePlanningReport()
    #report.add_filter(domain='gripper')
    #report.add_filter(lambda item: item['expanded'] == '21')
    #report.set_grouping('config', 'domain', 'problem')
    data = report.data
    report.write()
    

