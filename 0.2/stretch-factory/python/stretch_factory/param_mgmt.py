import stretch_body.hello_utils as hello_utils
import importlib
import click

def generate_user_params_from_template(model_name, fleet_dir=None):
    param_module_name = 'stretch_body.robot_params_' + model_name
    user_params = getattr(importlib.import_module(param_module_name), 'user_params_template')
    user_header = getattr(importlib.import_module(param_module_name), 'user_params_header')
    hello_utils.write_fleet_yaml('stretch_user_params.yaml',user_params,fleet_dir,user_header)

def generate_configuration_params_from_template(model_name, batch_name, robot_serial_no, fleet_dir=None):
    param_module_name = 'stretch_body.robot_params_' + model_name
    config_params = getattr(importlib.import_module(param_module_name), 'configuration_params_template')
    config_header = getattr(importlib.import_module(param_module_name), 'configuration_params_header')
    config_params['robot']['batch_name']=batch_name
    config_params['robot']['serial_no']=robot_serial_no
    hello_utils.write_fleet_yaml('stretch_configuration_params.yaml', config_params, fleet_dir,config_header)

def copy_over_params(dest_dict, src_dict,dest_dict_name='',src_dict_name=''):
    """
    Copy atomic values (list, numbers, strings) from src to dest
    Only if the key is found in the dest dict and types are the same
    """
    for k in dest_dict.keys():
        if k in src_dict:
            if type(src_dict[k])==type(dest_dict[k]):
                if type(src_dict[k])==dict:
                    copy_over_params(dest_dict[k],src_dict[k],dest_dict_name+'.'+str(k), src_dict_name+'.'+str(k))
                else:
                    dest_dict[k]=src_dict[k]
            else:
                click.secho('Migration error. Type mismatch for key %s during copy from %s to %s. From type %s. To type %s'%(k,src_dict_name,dest_dict_name,str(type(src_dict[k])),str(type(dest_dict[k])))
                            , fg="red")
                print('Values Src | Dest: ',src_dict[k],dest_dict[k])
        else:
            click.secho('Migration error. Parameter %s not found during copy from %s to %s'%(k,src_dict_name,dest_dict_name), fg="red")


def param_change_check(new_dict,prior_dict,num_warnings,new_dict_name,prior_dict_name,whitelist=[]):
    for k in new_dict.keys():
        if k in prior_dict:
            if type(new_dict[k])==dict:
                    num_warnings=param_change_check(new_dict[k],prior_dict[k],num_warnings,new_dict_name+'.'+k,prior_dict_name+'.'+k,whitelist)
            else:
                if new_dict[k]!=prior_dict[k]:
                    whitelisted = False
                    whitelist_name = prior_dict_name + '.' + k
                    whitelist_name = whitelist_name[whitelist_name.find('.') + 1:]  # eg robot.batch_name
                    for w in whitelist:
                        if w == whitelist_name:
                            whitelisted = True
                    if not whitelisted:
                        click.secho('Warning. Value change in %s from %s to %s'%(whitelist_name,prior_dict[k],new_dict[k]), fg="yellow")
                        num_warnings=num_warnings+1
    return num_warnings


def param_added_check(new_dict,prior_dict,num_warnings,new_dict_name,prior_dict_name,whitelist=[]):
    for k in new_dict.keys():
        if k in prior_dict:
            if type(new_dict[k])==dict:
                    num_warnings=param_added_check(new_dict[k],prior_dict[k],num_warnings,new_dict_name+'.'+k,prior_dict_name+'.'+k,whitelist)
        else:
            whitelisted = False
            whitelist_name = prior_dict_name+'.'+k
            whitelist_name = whitelist_name[whitelist_name.find('.') + 1:]  # eg robot.batch_name
            for w in whitelist:
                if w == whitelist_name:
                    whitelisted = True
            if not whitelisted:
                click.secho('Warning. Parameter introduced: %s'%whitelist_name, fg="yellow")
                num_warnings=num_warnings+1
    return num_warnings

def param_dropped_check(new_dict,prior_dict,num_warnings,new_dict_name,prior_dict_name,whitelist=[]):
    for k in prior_dict.keys():
        if k in new_dict:
            if type(prior_dict[k])==dict:
                    num_warnings=param_dropped_check(new_dict[k],prior_dict[k],num_warnings,new_dict_name+'.'+k,prior_dict_name+'.'+k,whitelist)
        else:
            whitelisted=False
            whitelist_name = prior_dict_name+'.'+k
            whitelist_name=whitelist_name[whitelist_name.find('.')+1:] #eg robot.batch_name
            for w in whitelist:
                if w==whitelist_name:
                    whitelisted=True
            if not whitelisted:
                click.secho('Warning. Parameter %s dropped'%whitelist_name, fg="yellow")
                num_warnings=num_warnings+1
    return num_warnings

# ####################### Contact Thresh Conversion ##############
# Read in old PseudoN units and write back out as effort_pct
# Remove the old parameters from the YAML as well

def contacts_pseudo_N_to_effort_pct(force_N_per_A, iMax_pos, iMax_neg,contacts_pseudo_N , i_feedforward):
    contacts_A = [(contacts_pseudo_N[0]/force_N_per_A)+i_feedforward, (contacts_pseudo_N[1]/force_N_per_A)+i_feedforward]
    return [100*contacts_A[0]/abs(iMax_neg), 100*contacts_A[1]/iMax_pos]

def convert_joint_contact_thresh(stepper_name,joint_name, N, D, i_feedforward):
    # Convert arm configuration data
    modified=False

    if joint_name not in D:
        return D, False
    if 'contact_models' not in D[joint_name]:
        D[joint_name]['contact_models']={'effort_pct':{}}

    iMax_pos=N[stepper_name]['gains']['iMax_pos']
    iMax_neg=N[stepper_name]['gains']['iMax_neg']
    if 'homing_force_N' in D[joint_name]:
        D[joint_name]['contact_models']['effort_pct']['contact_thresh_homing'] = contacts_pseudo_N_to_effort_pct(N[joint_name]['force_N_per_A'],
                                                     iMax_pos,
                                                     iMax_neg,
                                                     D[joint_name]['homing_force_N'],i_feedforward)

        print("Converted [%f, %f] to [%f, %f] for %s of %s"%(D[joint_name]['homing_force_N'][0],D[joint_name]['homing_force_N'][1],
                                                       D[joint_name]['contact_models']['effort_pct']['contact_thresh_homing'][0],
                                                       D[joint_name]['contact_models']['effort_pct']['contact_thresh_homing'][1], 'homing_force_N', joint_name))
        D[joint_name].pop('homing_force_N')
        modified=True

    if 'contact_thresh_N' in D[joint_name]:
        D[joint_name]['contact_models']['effort_pct']['contact_thresh_default'] = contacts_pseudo_N_to_effort_pct(N[joint_name]['force_N_per_A'],
                                                     iMax_pos,
                                                     iMax_neg,
                                                     D[joint_name]['contact_thresh_N'],i_feedforward)

        print("Converted [%f, %f] to [%f, %f] for %s of %s" % (
        D[joint_name]['contact_thresh_N'][0], D[joint_name]['contact_thresh_N'][1],
        D[joint_name]['contact_models']['effort_pct']['contact_thresh_default'][0],
        D[joint_name]['contact_models']['effort_pct']['contact_thresh_default'][1], 'contact_thresh_N',joint_name))
        D[joint_name].pop('contact_thresh_N')
        modified = True

    if 'contact_thresh_max_N' in D[joint_name]:
        D[joint_name].pop('contact_thresh_max_N')
        modified = True
    return D, modified

def migrate_contact_params_RE1V0(fleet_path, fleet_id):
    hello_utils.set_fleet_directory(fleet_path, fleet_id)
    U = hello_utils.read_fleet_yaml('stretch_user_params.yaml')
    C = hello_utils.read_fleet_yaml('stretch_configuration_params.yaml')
    if (len(U) == 0 or len(C) == 0):  # Empty file (corrupted)
        click.secho('Warning. Robot parameter data not found', fg="yellow")
        return False
    import stretch_body.robot_params_RE1V0
    N = stretch_body.robot_params_RE1V0.nominal_params


    C1, m1 = convert_joint_contact_thresh('hello-motor-arm','arm',N,C,C['arm']['i_feedforward'])
    C2, m2 = convert_joint_contact_thresh('hello-motor-lift', 'lift', N, C1,C['lift']['i_feedforward'])
    if m1 or m2:
        click.secho('Updated stretch_configuration_params.yaml.', fg="green")
        hello_utils.write_fleet_yaml('stretch_configuration_params.yaml', C2, None, stretch_body.robot_params_RE1V0.configuration_params_header)

    if 'arm' in U and 'i_feedforward' in U['arm']:
        i_feedforward = U['arm']['i_feedforward']
    else:
        i_feedforward=C['arm']['i_feedforward']
    U1, m3 = convert_joint_contact_thresh('hello-motor-arm','arm',N,U,i_feedforward)

    if 'lift' in U and 'i_feedforward' in U['lift']:
        i_feedforward = U['lift']['i_feedforward']
    else:
        i_feedforward = C['lift']['i_feedforward']
    U2, m4 = convert_joint_contact_thresh('hello-motor-lift', 'lift', N, U1,i_feedforward)

    #Do a bit of cleanup
    if 'tool_params' in U2:
        U2.pop('tool_params')
        m4=True

    if 'factory_params' in U2:
        U2.pop('factory_params')
        m4=True

    if m3 or m4:
        click.secho('Updated stretch_user_params.yaml.', fg="green")
        hello_utils.write_fleet_yaml('stretch_user_params.yaml', U2, None,stretch_body.robot_params_RE1V0.configuration_params_header)
    if not m1 and not m2 and not m3 and not m4:
        click.secho('Contact parameter migration not required.', fg="green")
    return True


# #########################################################################################

#Todo: make generic for future migrations / parameter org changes. Do we version parameter orgs?
def migrate_params_RE1V0(fleet_path, fleet_id, dropped_user_params):
    """
    The parameter organization has changed between RE1P0 and RE1P5 in the following ways
    1. stretch_re1_user_params.yaml is now named stretch_user_params.yaml
    2. stretch_configuration_params.yaml is introduced
    3. stretch_factory_params.yaml is deprecated
    4. Parameter precendence has changed as described in stretch_body.robot_params.py

    Migration algorithm:
    ---------------------------------

    #Construct the original robot_params dictionary without user data (O)
    #Construct a new Configuration Params dictionary (C)
    #Copy data from O to C provided the parameter exists in C (otherwise drop the parameter)
    #Save C to stretch_configuration_params.yaml
    #Copy the original user params to stretch_user_params.yaml


    """
    #Point to the data to be migrated
    hello_utils.set_fleet_directory(fleet_path, fleet_id)
    #Get the original parameter dictionaries
    U = hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
    F = hello_utils.read_fleet_yaml(U.get('factory_params', ''))
    if (len(U)==0 or len(F)==0):#Empty file (corrupted)
        return None, None, None
    #Construct the original robot_params dictionary without user data (O)
    import stretch_body.robot_params_RE1V0
    import copy
    O = copy.deepcopy(F)
    hello_utils.overwrite_dict(O, stretch_body.robot_params_RE1V0.factory_params_deprecated)
    for external_params_module in U.get('params', []):
        hello_utils.overwrite_dict(O, getattr(importlib.import_module(external_params_module), 'params'))
    #Construct a new Configuration Params dictionary (C)
    generate_configuration_params_from_template(model_name='RE1V0', batch_name=O['robot']['batch_name'], robot_serial_no=O['robot']['serial_no'], fleet_dir=None)
    C = hello_utils.read_fleet_yaml('stretch_configuration_params.yaml')
    # Manually copy over newly introduced params so doesn't generate an error )
    if 'model_name' not in O['robot']:
        O['robot']['model_name'] = C['robot']['model_name']
    if 'd435i' not in O['robot']:
        O['robot']['d435i'] = C['robot']['d435i'].copy()
    #Now copy over rest of O data to C
    copy_over_params(C,O,'NewParams','OldParams')
    hello_utils.write_fleet_yaml('stretch_configuration_params.yaml', C, None,stretch_body.robot_params_RE1V0.configuration_params_header)

    #Construct the full prior param dict R
    R=copy.deepcopy(O)
    hello_utils.overwrite_dict(R,U)

    #Write user params to new yaml
    generate_user_params_from_template('RE1V0')
    UU = hello_utils.read_fleet_yaml('stretch_user_params.yaml')
    #Cleanup old user yaml
    for d in dropped_user_params:
        if U.has_key(d):
            U.pop(d)
    #Construct new user YAML where the old user yaml takes precendence
    hello_utils.overwrite_dict(overwritee_dict=UU, overwriter_dict=U)
    hello_utils.write_fleet_yaml('stretch_user_params.yaml', UU, None,stretch_body.robot_params_RE1V0.user_params_header)
    return O,UU,R #Return

