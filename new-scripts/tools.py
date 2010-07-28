import os
import sys
import shutil
import re
from optparse import OptionParser, OptionValueError, BadOptionError, Option
from shutil import *

from external import argparse
from external.configobj import ConfigObj


def divide_list(seq, size):
    '''
    >>> divide_list(range(10), 4)
    [[0, 1, 2, 3], [4, 5, 6, 7], [8, 9]]
    '''
    return [seq[i:i+size] for i  in range(0, len(seq), size)]
    

def overwrite_dir(dir):
    if os.path.exists(dir):
        if not os.path.exists(os.path.join(dir, 'run')):
            msg = 'The experiment directory "%s" ' % dir
            msg += 'is not empty, do you want to overwrite it? (Y/N): '
            answer = raw_input(msg).upper().strip()
            if not answer == 'Y':
                sys.exit('Aborted')
        shutil.rmtree(dir)
    os.makedirs(dir)
    
    
def natural_sort(alist):
    """Sort alist alphabetically, but special-case numbers to get
    file2.txt before file10.ext."""
    def to_int_if_number(text):
        if text.isdigit():
            return int(text)
        else:
            return text
    def extract_numbers(text):
        parts = re.split("([0-9]+)", text)
        return map(to_int_if_number, parts)
    alist.sort(key=extract_numbers)


def listdir(path):
    return [filename for filename in os.listdir(path)
            if filename != ".svn"]


def find_file(basenames, dir="."):
    for basename in basenames:
        path = os.path.join(dir, basename)
        if os.path.exists(path):
            return path
    raise IOError("none found in %r: %r" % (dir, basenames))
    
    
def convert_to_correct_type(val):
    '''
    Safely evaluate an expression node or a string containing a Python expression. 
    The string or node provided may only consist of the following Python literal 
    structures: strings, numbers, tuples, lists, dicts, booleans, and None.
    '''
    import ast
    try:
        val = ast.literal_eval(val)
    except (ValueError, SyntaxError):
        pass
    return val
    

class ExtOption(Option):
    '''
    This extended option allows comma-separated commandline options
    Strings that represent integers are automatically converted to ints
    
    Example:
    python myprog.py -a foo,bar -b aha -a oho,3
    '''

    ACTIONS = Option.ACTIONS + ("extend",)
    STORE_ACTIONS = Option.STORE_ACTIONS + ("extend",)
    TYPED_ACTIONS = Option.TYPED_ACTIONS + ("extend",)
    ALWAYS_TYPED_ACTIONS = Option.ALWAYS_TYPED_ACTIONS + ("extend",)
    
    def try_str_to_int(self, string):
        try:
            integer = int(string)
            return integer
        except ValueError:
            return string

    def convert_list(self, list):
        return map(self.try_str_to_int, list)

    def take_action(self, action, dest, opt, value, values, parser):
        if action == "extend":
            lvalue = value.split(",")
            values.ensure_value(dest, []).extend(self.convert_list(lvalue))
        else:
            Option.take_action(
                self, action, dest, opt, value, values, parser)
                
                
class Properties(ConfigObj):
    def __init__(self, *args, **kwargs):
        kwargs['unrepr'] = True
        ConfigObj.__init__(self, *args, **kwargs)
        
                
                
def updatetree(src, dst, symlinks=False, ignore=None):
    '''
    Copies the contents from src onto the tree at dst, overwrites files with the
    same name
    
    Code taken and expanded from python docs
    '''
    
    names = os.listdir(src)
    if ignore is not None:
        ignored_names = ignore(src, names)
    else:
        ignored_names = set()

    if not os.path.exists(dst):
        os.makedirs(dst)
        
    errors = []
    for name in names:
        if name in ignored_names:
            continue
        srcname = os.path.join(src, name)
        dstname = os.path.join(dst, name)
        try:
            if symlinks and os.path.islink(srcname):
                linkto = os.readlink(srcname)
                os.symlink(linkto, dstname)
            elif os.path.isdir(srcname):
                copytree(srcname, dstname, symlinks, ignore)
            else:
                copy2(srcname, dstname)
            # XXX What about devices, sockets etc.?
        except (IOError, os.error), why:
            errors.append((srcname, dstname, str(why)))
        # catch the Error from the recursive copytree so that we can
        # continue with other files
        except Error, err:
            errors.extend(err.args[0])
    try:
        copystat(src, dst)
    except WindowsError:
        # can't copy file access times on Windows
        pass
    except OSError, why:
        errors.extend((src, dst, str(why)))
    if errors:
        raise Error(errors)
        
        
class ArgParser(argparse.ArgumentParser):
    def __init__(self, *args, **kwargs):
        argparse.ArgumentParser.__init__(self, *args, add_help=False,
                formatter_class=argparse.ArgumentDefaultsHelpFormatter, **kwargs)
                
    def set_help_active(self):
        self.add_argument(
                '-h', '--help', action='help', default=argparse.SUPPRESS,
                help=('show this help message and exit'))
      
    def directory(self, string):
        if not os.path.isdir(string):
            msg = "%r is not an evaluation directory" % string
            raise argparse.ArgumentTypeError(msg)
        return string
